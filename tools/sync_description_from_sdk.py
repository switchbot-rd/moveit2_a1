#!/usr/bin/env python3
"""
从 armsdk 的权威 URDF 生成 ROS 描述包用的 body URDF。

设计原则：
  * armsdk 的 onero_sdk/urdf/A1/a1_r.urdf 是【唯一事实来源】（SolidWorks 导出）
  * 本脚本只做三件事：改 mesh 路径、去掉 MuJoCo 专用标签、改 robot name
  * 【绝不】修改任何关节的 origin / axis / parent / child / limit —— 并会自校验

用法：
    python3 sync_description_from_sdk.py \
        --sdk-urdf /path/to/armsdk/onero_sdk/urdf/A1/a1_r.urdf \
        --out      /path/to/onero_a1_description/urdf/onero_a1_body.urdf
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    from tools.collision_meshes import DESCRIPTION_DIR, MESH_URI_PREFIX, apply_collision_meshes
except ModuleNotFoundError:
    from collision_meshes import DESCRIPTION_DIR, MESH_URI_PREFIX, apply_collision_meshes

# 关节/连杆的"几何身份"字段 —— 这些绝不允许被本脚本改动
GEOMETRY_ATTRS = ("name", "type")
ORIGIN_ATTRS = ("xyz", "rpy")
LIMIT_ATTRS = ("lower", "upper", "effort", "velocity")
AXIS_ATTRS = ("xyz",)


def _snapshot(root: ET.Element) -> dict:
    """把 URDF 的几何要素抽出来做指纹，用于自校验。"""
    snap = {"joints": {}, "links": sorted(l.get("name") for l in root.findall("link"))}

    for j in root.findall("joint"):
        name = j.get("name")
        origin = j.find("origin")
        limit = j.find("limit")
        axis = j.find("axis")
        snap["joints"][name] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "origin": {a: (origin.get(a) if origin is not None else None) for a in ORIGIN_ATTRS},
            "axis": {a: (axis.get(a) if axis is not None else None) for a in AXIS_ATTRS},
            "limit": {a: (limit.get(a) if limit is not None else None) for a in LIMIT_ATTRS},
        }
    return snap


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdk-urdf", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--robot-name", default="onero_a1")
    ap.add_argument("--mesh-prefix", default="package://onero_a1_description/meshes/A1_r")
    args = ap.parse_args()

    if not args.sdk_urdf.is_file():
        print(f"ERROR: SDK URDF 不存在: {args.sdk_urdf}", file=sys.stderr)
        return 2

    tree = ET.parse(args.sdk_urdf)
    root = tree.getroot()

    before = _snapshot(root)

    # ── 只做这三项变换 ──────────────────────────────────────────────
    # 1) 去掉 MuJoCo 专用标签（ROS 不认，留着会有警告）
    for tag in ("mujoco", "gazebo"):
        for elem in root.findall(tag):
            root.remove(elem)

    # 2) 重写 mesh 路径：../../meshes/A1/A1_r/xxx.STL -> package://<pkg>/meshes/A1_r/xxx.STL
    n_mesh = 0
    collision_meshes = set(root.findall('.//collision/geometry/mesh'))
    collision_sources = {}
    for mesh in root.iter("mesh"):
        old = mesh.get("filename", "")
        if not old:
            continue
        fname = old.replace("\\", "/").rsplit("/", 1)[-1]
        mesh.set("filename", f"{args.mesh_prefix}/{fname}")
        if mesh in collision_meshes and mesh.get('filename').startswith(MESH_URI_PREFIX):
            relative = mesh.get('filename')[len(MESH_URI_PREFIX):]
            collision_sources[relative] = args.sdk_urdf.parent / old.replace('\\', '/')
        n_mesh += 1

    # This entry point does not copy SDK meshes; verify them before replacing collisions.
    apply_collision_meshes(root, DESCRIPTION_DIR, source_paths=collision_sources)

    # 3) 改 robot name
    root.set("name", args.robot_name)

    after = _snapshot(root)

    # ── 自校验：几何身份必须逐字节一致 ──────────────────────────────
    problems = []
    if before["links"] != after["links"]:
        problems.append("link 列表发生了变化")
    if set(before["joints"]) != set(after["joints"]):
        problems.append("joint 列表发生了变化")
    for name, bj in before["joints"].items():
        aj = after["joints"].get(name)
        if aj is None:
            problems.append(f"{name}: 丢失")
            continue
        for key in ("type", "parent", "child", "origin", "axis", "limit"):
            if bj[key] != aj[key]:
                problems.append(f"{name}.{key}: {bj[key]} -> {aj[key]}")

    if problems:
        print("ERROR: 几何身份校验失败，已中止（不会写出文件）:", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        return 1

    # ── 写出 ────────────────────────────────────────────────────────
    args.out.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(args.out, encoding="utf-8", xml_declaration=True)

    print(f"✓ 已生成: {args.out}")
    print(f"  - 来源:        {args.sdk_urdf}")
    print(f"  - robot name:  {args.robot_name}")
    print(f"  - mesh 重写:   {n_mesh} 处")
    print(f"  - 关节数:      {len(before['joints'])}")
    print(f"  - 连杆数:      {len(before['links'])}")
    print("  - 几何身份校验: 通过（origin/axis/limit/parent/child 全部未改动）")
    return 0


if __name__ == "__main__":
    sys.exit(main())

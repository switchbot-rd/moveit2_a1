#!/usr/bin/env python3
"""Generate checked collision STL assets with the separately built CGAL tool."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simplifier', type=Path, required=True, help='collision_mesh_simplifier 可执行文件')
    parser.add_argument('--max-error-mm', type=float, default=2., help='双向表面误差上限，毫米，默认 2')
    args = parser.parse_args()
    if not math.isfinite(args.max_error_mm) or not 0. < args.max_error_mm <= 2.:
        parser.error('--max-error-mm 必须大于 0 且不超过 2')
    tolerance = args.max_error_mm/1000.
    executable = args.simplifier.resolve()
    meshes = ROOT/'src/onero_a1_description/meshes'
    output = meshes/'collision'
    output.mkdir(exist_ok=True)
    sources = sorted(path for folder in ('A1_l', 'A1_r', 'grippers/woan_picker')
                     for path in (meshes/folder).glob('*.STL'))
    entries, retained = {}, []
    with tempfile.TemporaryDirectory(prefix='onero_collision_') as temporary:
        temporary = Path(temporary)

        def generate(source):
            relative = source.relative_to(meshes)
            if source.name in ('Link_L3.STL', 'Link_R3.STL'):
                # MTC approaches pass close to the bottle along the whole
                # fingertip outline, not only the flat pad. Keep that outline exact.
                print(f'保留原网格：{relative}；保留指尖接近水瓶的完整轮廓', flush=True)
                return str(relative), None
            candidate = temporary/relative
            candidate.parent.mkdir(parents=True, exist_ok=True)
            result = subprocess.run([str(executable), str(source), str(candidate), str(tolerance)],
                                    capture_output=True, text=True, timeout=240.)
            if result.returncode:
                print(f'保留原网格：{relative}；{result.stderr.strip()}', flush=True)
                return str(relative), None
            detail = json.loads(result.stdout)
            destination = output/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(candidate.read_bytes())
            detail.update(source_sha256=digest(source), collision_sha256=digest(destination),
                          collision=str(destination.relative_to(meshes)))
            print(f'{relative}: {detail["triangles_before"]} → {detail["triangles_after"]}', flush=True)
            return str(relative), detail

        with ThreadPoolExecutor(max_workers=3) as executor:
            for relative, detail in executor.map(generate, sources):
                if detail is None:
                    retained.append(relative)
                else:
                    entries[relative] = detail
    manifest = dict(format_version=1, max_surface_error_m=tolerance,
                    method='CGAL 5.6 constrained edge collapse; finger mechanism edges and pad surfaces retained; '
                           'bidirectional whole-triangle envelope check against original STL after export',
                    meshes=entries, retained_original=retained)
    temporary = output/'manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(output/'manifest.json')


if __name__ == '__main__':
    main()

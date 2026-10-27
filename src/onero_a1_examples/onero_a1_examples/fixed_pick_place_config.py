"""Validated, world-fixed settings for the bottle A-to-B example."""
from dataclasses import asdict, dataclass
import math
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .bottle_scene import matrix_pose
from .support import MoveItError


@dataclass
class TaskConfig:
    frame: str
    arm: str
    pick_position: list
    place_position: list
    grasp_rpy: list
    transfer_height: float
    diameter: float = .05
    bottle_height: float = .18
    grasp_height: float = 0.
    tcp_offset: list = None
    grasp_opening: float = None
    open_opening: float = 1.
    approach_direction: list = None
    approach_distance: float = .05
    lift_distance: float = .03
    retreat_distance: float = .05
    carry_kp: float = 120.
    orientation_tolerance: float = .1
    position_tolerance: float = .015
    pick_support: str = None
    place_support: str = None
    ready_joints: list = None

    @classmethod
    def load(cls, path, *, recording=False):
        try:
            data = yaml.safe_load(Path(path).read_text())
        except (OSError, yaml.YAMLError) as exc:
            raise MoveItError(f'无法读取抓放配置：{exc}') from exc
        if not isinstance(data, dict):
            raise MoveItError('抓放配置必须是 YAML 字典')
        unknown = set(data)-set(cls.__dataclass_fields__)
        if unknown:
            raise MoveItError('未知抓放配置项：'+', '.join(sorted(map(str, unknown))))
        try:
            result = cls(**data)
        except TypeError as exc:
            raise MoveItError(f'抓放配置缺少必填项：{exc}') from exc
        result.validate(recording=recording)
        return result

    def validate(self, *, recording=False):
        if self.frame != 'world' or self.arm not in ('left', 'right'):
            raise MoveItError('配置 frame 必须为 world，arm 必须为 left 或 right')

        def scalar(name, low=-math.inf, high=math.inf, strict=False):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not low <= value <= high or strict and value == low):
                raise MoveItError(f'{name} 必须是范围内的有限数值：{low}..{high}')
            setattr(self, name, float(value))

        def vector(name, size=3, optional=False):
            value = getattr(self, name)
            if optional and value is None:
                return
            if (not isinstance(value, list) or len(value) != size or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
                raise MoveItError(f'{name} 必须包含 {size} 个有限数值')
            setattr(self, name, list(map(float, value)))

        for name in ('pick_position', 'place_position'):
            vector(name)
        if self.grasp_rpy is None and not recording:
            raise MoveItError('先填写 grasp_rpy，或使用 --record-orientation 记录当前 Link7 朝向')
        vector('grasp_rpy', optional=recording)
        vector('tcp_offset', optional=True)
        vector('ready_joints', 7, optional=True)
        if self.approach_direction is None:
            raise MoveItError('请配置 world 中指向物体的 approach_direction')
        vector('approach_direction')
        direction = np.array(self.approach_direction)
        length = np.linalg.norm(direction)
        if length < 1e-6 or abs(direction[2]) > 1e-6:
            raise MoveItError('首版侧夹水瓶要求 approach_direction 为非零水平向量')
        self.approach_direction = (direction/length).tolist()
        for name in ('diameter', 'bottle_height', 'approach_distance', 'lift_distance', 'retreat_distance'):
            scalar(name, 0., strict=True)
        scalar('carry_kp', 0., 500., strict=True)
        scalar('orientation_tolerance', 0., .2, strict=True)
        scalar('position_tolerance', 0., .02, strict=True)
        scalar('open_opening', 0., 1., strict=True)
        if self.grasp_opening is not None:
            scalar('grasp_opening', 0., self.open_opening, strict=True)
            if self.grasp_opening == self.open_opening:
                raise MoveItError('open_opening 必须大于 grasp_opening')
        scalar('grasp_height')
        if abs(self.grasp_height) > self.bottle_height/2.-.025:
            raise MoveItError('grasp_height 必须位于瓶身内，并离上下边缘至少 25 mm')
        scalar('transfer_height')
        if self.transfer_height < max(self.pick_position[2], self.place_position[2])+self.lift_distance:
            raise MoveItError('transfer_height 必须高于两个瓶身中心至少 lift_distance')
        if np.linalg.norm(np.array(self.pick_position)-self.place_position) < 1e-6:
            raise MoveItError('取料点 A 与放料点 B 必须不同')
        for name in ('pick_support', 'place_support'):
            value = getattr(self, name)
            if value is not None and (not isinstance(value, str) or not value.strip()):
                raise MoveItError(f'{name} 必须为空或场景中的支撑面对象 ID')

    def dictionary(self):
        return asdict(self)

    def object_pose(self, place=False):
        transform = np.eye(4)
        transform[:3, 3] = self.place_position if place else self.pick_position
        return matrix_pose(transform)

    def grasp_pose(self, offset):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', self.grasp_rpy).as_matrix()
        center = np.array(self.pick_position)+[0., 0., self.grasp_height]
        transform[:3, 3] = center-transform[:3, :3] @ offset
        return matrix_pose(transform)

"""碰撞接触点可视化：持续查询状态，在 RViz 标出接触位置和碰撞对象，不产生运动。"""
from copy import deepcopy
import time

import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .planning_scene import set_obstacle
from .support import positive, run_example


MARKER_TOPIC = '/collision_contacts'


def configure(parser):
    parser.add_argument('--target', help='可选 SRDF 姿态名；默认持续查询当前实测姿态')
    parser.add_argument('--rate', type=positive, default=5., help='查询频率，Hz，默认 5')
    parser.add_argument('--marker-size', type=positive, default=.02, help='接触点直径和文字高度，米')
    parser.add_argument('--demo-obstacle', action='store_true', help='在所查姿态的法兰处添加临时球体')
    parser.add_argument('--obstacle-radius', type=positive, default=.04, help='演示球体半径，米，默认 0.04')


def contact_markers(contacts, *, size, lifetime):
    # Each message is a complete snapshot, including the no-collision case.
    result = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
    for index, contact in enumerate(contacts):
        marker = Marker(
            header=Header(frame_id=contact.header.frame_id), ns='collision_contacts',
            id=2*index, type=Marker.SPHERE, action=Marker.ADD,
            lifetime=Duration(seconds=lifetime).to_msg())
        marker.pose.position = deepcopy(contact.position)
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r = marker.color.b = marker.color.a = 1.
        result.markers.append(marker)

        label = deepcopy(marker)
        label.id += 1
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.z += 1.5*size
        label.color.g = 1.
        label.text = f'{contact.contact_body_1} ↔ {contact.contact_body_2} ({contact.depth*1000:.1f} mm)'
        result.markers.append(label)
    return result


def clear_markers(node):
    publisher = getattr(node, 'contact_publisher', None)
    if publisher is not None:
        publisher.publish(contact_markers([], size=.02, lifetime=0.))
        publisher.wait_for_all_acked(Duration(seconds=.5))


def example(node):
    node.contact_publisher = node.create_publisher(
        MarkerArray, MARKER_TOPIC, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    target = node.named_target(node.args.target) if node.args.target else {}
    if node.args.demo_obstacle:
        link = 'right_Link7' if node.arm_side == 'both' else node.tip_link
        set_obstacle(node, node.fk({**node.start, **target}, link=link))
        print('已在所查姿态的法兰处添加临时球体；退出时删除。')
    print(f'RViz MarkerArray：{MARKER_TOPIC}；紫色球为接触点，文字为碰撞对象和穿透深度。')
    if target:
        print(f'查询命名姿态 {node.args.target}；RViz 场景机器人仍显示实测姿态。')
    print('按 Ctrl+C 退出。')
    previous = None
    try:
        while rclpy.ok():
            started = time.monotonic()
            q = {**node.state(), **target}
            # Always request full ContactInformation, even if a local planning
            # validator (which only returns body names) is installed later.
            result = node.call(GetStateValidity, '/check_state_validity', GetStateValidity.Request(
                robot_state=node.robot_state(q), group_name=node.group))
            node.contact_publisher.publish(contact_markers(
                result.contacts, size=node.args.marker_size, lifetime=max(1., 3./node.args.rate)))
            pairs = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in result.contacts})
            summary = (result.valid, len(result.contacts), pairs)
            if summary != previous:
                print(f'状态有效：{result.valid}；接触点：{len(result.contacts)}')
                for first, second in pairs:
                    print(f'碰撞接触：{first} ↔ {second}')
                previous = summary
            time.sleep(max(0., 1./node.args.rate-(time.monotonic()-started)))
    except KeyboardInterrupt:
        pass


def main(args=None):
    return run_example('collision_contacts', __doc__, configure, example, args,
                       query_only=True, allow_both=True, cleanup=clear_markers)

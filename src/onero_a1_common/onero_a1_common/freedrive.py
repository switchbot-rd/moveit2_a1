"""Coordinate hand guiding with the bridge and the selected trajectory controllers."""
import math
import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from std_srvs.srv import SetBool, Trigger


def _boolean(node, target, name, error):
    values = node.call(GetParameters, target+'/get_parameters', GetParameters.Request(names=[name])).values
    if len(values) != 1 or values[0].type != 1:
        raise error(f'无法读取 {target} 的 {name}')
    return values[0].bool_value


def _switch(node, names, activate, error):
    if not names:
        return
    states = {controller.name: controller.state for controller in node.call(
        ListControllers, '/controller_manager/list_controllers', ListControllers.Request()).controller}
    expected = 'active' if activate else 'inactive'
    names = [name for name in names if states.get(name) != expected]
    if not names:
        return
    request = SwitchController.Request(
        activate_controllers=names if activate else [],
        deactivate_controllers=[] if activate else names,
        strictness=SwitchController.Request.STRICT, activate_asap=True,
        timeout=Duration(sec=3))
    if not node.call(SwitchController, '/controller_manager/switch_controller', request).ok:
        raise error('轨迹控制器切换失败：'+', '.join(names))


def run_freedrive(node, *, damping, duration, execute, error):
    if not math.isfinite(damping) or not 0. <= damping <= 5.:
        raise error('拖动阻尼必须在 [0, 5] 内')
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise error('拖动持续时间必须为有限正数')
    sessions = node._startup_sessions()
    for session in sessions:
        bridge = session['bridge']
        if session['identity']['simulate']:
            raise error('零重力拖动仅支持 real 模式')
        if not _boolean(node, bridge, 'gravity_compensation', error):
            raise error('零重力拖动需要 gravity_compensation:=true')
    controllers = node.call(ListControllers, '/controller_manager/list_controllers',
                            ListControllers.Request()).controller
    states = {controller.name: controller.state for controller in controllers}
    for name in node.arm_controllers:
        # A failed scene check leaves trajectory controllers unloaded. The
        # bridge can still guide/hold the arm so the pose can be corrected.
        if name not in states:
            continue
        if states[name] not in ('active', 'inactive'):
            raise error(f'轨迹控制器 {name} 未就绪')
        if _boolean(node, '/'+name, 'set_last_command_interface_value_as_state_on_activation', error):
            raise error('控制器需要从实测位置激活，请更新并重启服务')
    if not execute:
        print('拖动条件检查通过；添加 --execute 进入零重力拖动模式。')
        return
    entered, paused, failures = [], [], []
    try:
        for session, controller in zip(sessions, node.arm_controllers):
            bridge = session['bridge']
            result = node.call(SetParametersAtomically, bridge+'/set_parameters_atomically',
                               SetParametersAtomically.Request(parameters=[
                                   Parameter('freedrive_damping', value=float(damping)).to_parameter_msg()])).result
            if not result.successful:
                raise error(result.reason)
            result = node.call(SetBool, bridge+'/freedrive', SetBool.Request(data=True))
            if not result.success:
                raise error(result.message)
            entered.append(bridge)
            if states.get(controller) == 'active':
                paused.append(controller)
        _switch(node, paused, False, error)
        print('已进入零重力拖动，可手动引导所选手臂；停止拖动后按 Ctrl+C 恢复当前位置保持。')
        deadline = time.monotonic()+duration if duration is not None else math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            for bridge in entered:
                response = node.call(Trigger, bridge+'/freedrive_heartbeat', Trigger.Request())
                if not response.success:
                    raise error(bridge+' 的拖动会话已结束')
            until = min(deadline, time.monotonic()+.25)
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=min(.05, max(0., until-time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        if paused:
            try:
                _switch(node, paused, False, error)
            except Exception as exc:
                failures.append(str(exc))
        for bridge in reversed(entered):
            try:
                result = node.call(SetBool, bridge+'/freedrive', SetBool.Request(data=False))
                if not result.success:
                    raise error(result.message)
            except Exception as exc:
                failures.append(str(exc))
        if paused and not failures:
            try:
                # Give the hardware interface fresh feedback before activation.
                # JTC is configured to initialize from that measured pose.
                until = time.monotonic()+.1
                while time.monotonic() < until and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=.02)
                _switch(node, paused, True, error)
            except Exception as exc:
                failures.append(str(exc))
        if failures:
            raise error('拖动退出或控制器恢复失败，请检查服务状态：'+'；'.join(failures))
    if any(name not in states for name in node.arm_controllers):
        print('已退出拖动，保持当前位置。恢复自动运动前，请重启服务并确认起始姿态检查通过。')
    else:
        print('已退出拖动，保持当前位置并恢复原先激活的轨迹控制器。')

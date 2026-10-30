"""Servo 键盘遥操作：按住 Shift 使能，松键/失焦停止，Esc 退出。"""
import argparse
import math
import sys

import rclpy
from onero_a1_common.moveit_client import MoveItClient, MoveItError
from onero_a1_common.gains import GainClient
from onero_a1_common.servo import KeyboardState, ServoClient


def speed(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0.:
        raise argparse.ArgumentTypeError('速度必须是有限正数')
    return value


class KeyboardWindow:
    def __init__(self, node, client):
        import tkinter as tk
        from tkinter import ttk
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise MoveItError('键盘控制需要图形桌面；请检查 DISPLAY / Wayland 会话') from exc
        self.node, self.client, self.keys = node, client, KeyboardState()
        self.pending, self.closed = {}, False
        self.held_keys, self.key_widgets = set(), {}
        self.switch_rearm = False
        self.root.title(f'A1 {node.arm_side} · Servo 遥操作')
        self.root.resizable(False, False)
        panel = ttk.Frame(self.root, padding=18)
        panel.grid()
        ttk.Label(panel, text='按住 Shift 操作', font=('TkDefaultFont', 14)).grid(sticky='w')
        rows = [('平移', 'W/S 前后 X    A/D 左右 Y    R/F 上下 Z'),
                ('旋转', 'U/O 绕 X       I/K 绕 Y        J/L 绕 Z'),
                ('夹爪', 'Z 张开，X 夹取；按住 Shift 操作，松键保持夹持'),
                ('停止', '松开 Shift 或方向键停止；空格暂停，Enter 恢复'),
                ('退出', 'Esc / 关闭窗口：停稳并恢复普通控制器'),
                ('坐标', f'{node.args.frame}；控制点为 Link7 法兰'),
                ('速度', f'{node.args.linear_speed:g} m/s，{node.args.angular_speed:g} rad/s'),
                ('姿态', '平移优先，接近奇异姿态时允许小幅调整朝向')]
        if node.dual:
            rows.append(('选臂', 'Tab 切换左臂 / 右臂 / 双臂；停稳后重新按 Shift 操作'))
        for index, (name, text) in enumerate(rows, 1):
            ttk.Label(panel, text=name+'  '+text).grid(row=index, sticky='w', pady=4)
        keyboard = ttk.Frame(panel)
        keyboard.grid(row=len(rows)+1, sticky='w', pady=(10,4))
        for row, names in enumerate((('shift','w','s','a','d','r','f'),
                                     ('u','o','i','k','j','l'), ('z','x','space','return','tab'))):
            for column, key in enumerate(names):
                label = tk.Label(keyboard, text={'shift':'Shift','space':'空格','return':'Enter'}.get(key,key.upper()),
                                 width=6, padx=3, pady=5, background='#e5e7eb', foreground='#1f2937')
                label.grid(row=row, column=column, padx=3, pady=3)
                self.key_widgets[key] = label
        self.gripper_status = tk.StringVar(value='夹爪：等待接管控制…')
        ttk.Label(panel, textvariable=self.gripper_status).grid(row=len(rows)+2, sticky='w', pady=4)
        self.status = tk.StringVar(value='等待接管控制…')
        ttk.Label(panel, textvariable=self.status, wraplength=540).grid(row=len(rows)+3, sticky='w', pady=(8, 0))
        self.root.bind('<KeyPress>', self.press)
        self.root.bind('<KeyRelease>', self.release)
        self.root.bind('<FocusOut>', self.focus_out)
        self.root.bind('<FocusIn>', lambda event: setattr(self.keys, 'focused', True))
        self.root.protocol('WM_DELETE_WINDOW', self.quit)

    def press(self, event):
        key = event.keysym.lower()
        first = key not in self.held_keys
        self.held_keys.add(key)
        if key in self.pending:
            self.root.after_cancel(self.pending.pop(key))
        if key == 'escape':
            self.quit()
        elif key == 'space':
            self.keys.stop()
            self.switch_rearm = False
        elif key == 'tab' and self.node.dual:
            if first and not self.client.switching:
                self.keys.stop()
                self.switch_rearm = True
                self.client.cycle_target()
        elif key == 'return' and not self.client.switching:
            self.keys.stop(pause=False)
            self.switch_rearm = False
        elif not self.client.switching:
            if self.switch_rearm and first and key in ('shift_l', 'shift_r'):
                self.keys.stop(pause=False)
                self.switch_rearm = False
            self.keys.pressed.add(key)
        self.update_highlights()

    def update_highlights(self):
        held = set(self.held_keys)
        if held & {'shift_l','shift_r'}:
            held.add('shift')
        for key, label in self.key_widgets.items():
            label.configure(background='#2563eb' if key in held else '#e5e7eb',
                            foreground='white' if key in held else '#1f2937')

    def release(self, event):
        key = event.keysym.lower()
        if key in self.pending:
            self.root.after_cancel(self.pending[key])
        def confirmed():
            self.pending.pop(key, None)
            self.keys.pressed.discard(key)
            self.held_keys.discard(key)
            self.update_highlights()
        # X11 autorepeat can emit a release/press pair; cancel only that
        # synthetic release, while real key releases stop within one UI tick.
        self.pending[key] = self.root.after(12, confirmed)

    def focus_out(self, _event):
        self.keys.focused = False
        self.keys.stop()
        self.switch_rearm = False
        self.held_keys.clear()
        self.update_highlights()

    def quit(self):
        self.keys.stop()
        self.held_keys.clear()
        self.update_highlights()
        self.closed = True
        try:
            self.client.send(self.values(), gripper=0)
        finally:
            self.root.quit()

    def values(self):
        return self.keys.velocity(self.node.args.linear_speed, self.node.args.angular_speed)

    def tick(self):
        if self.closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.)
            state = self.client.state
            selected = {'left':'左臂','right':'右臂','both':'双臂'}.get(self.client.control_target, '')
            self.root.title(f'A1 {selected} · Servo 遥操作'+(' · 切换中' if self.client.switching else ''))
            if self.client.session and state and state.get('session_id') != self.client.session['session_id']:
                self.keys.stop()
                self.status.set('会话已停止：'+state.get('error', '请退出并重新打开示例'))
            else:
                values = self.values()
                feedback = state.get('servo_status', '等待状态')
                if state.get('orientation_deviation_rad', 0.) > math.radians(1.):
                    feedback += f"；朝向调整约 {math.degrees(state['orientation_deviation_rad']):.1f}°"
                self.status.set(('已暂停，Enter 恢复。' if self.keys.paused else '就绪，按住 Shift 操作。')
                                +' Servo：'+feedback)
                direction = self.keys.gripper_direction()
                available = self.client.session and self.client.session.get('gripper_topic')
                self.gripper_status.set('夹爪：'+(state.get('gripper_error') or
                    ({1:'张开',-1:'闭合夹取',0:'保持夹持' if state.get('gripper_holding') else '保持开度'}[direction]
                     if available else '未配置夹爪')))
                self.client.send(values, gripper=direction)
            self.root.after(20, self.tick)
        except Exception as exc:
            self.keys.stop()
            self.status.set('控制中止：'+str(exc))
            self.quit()

    def run(self):
        self.root.update_idletasks()
        bridges = ['/left/onero_topic_bridge', '/right/onero_topic_bridge'] if self.node.dual else ['/onero_topic_bridge']
        for bridge in bridges:
            GainClient(self.node, bridge).set_kp(120.)
        self.node.get_logger().info('Servo 位置增益已生效：kp=120。')
        self.client.start()
        self.root.after(0, self.tick)
        self.root.mainloop()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=['left', 'right', 'both'])
    parser.add_argument('--frame', choices=['world', 'base_link'], default='world')
    parser.add_argument('--linear-speed', type=speed, default=.2, help='法兰线速度 m/s，默认 0.2')
    parser.add_argument('--angular-speed', type=speed, default=.5, help='法兰角速度 rad/s，默认 0.5')
    parser.add_argument('--execute', action='store_true', help='接管 Servo 并打开键盘窗口；默认只检查环境')
    supplied = list(args) if args is not None else sys.argv[1:]
    split = supplied.index('--ros-args') if '--ros-args' in supplied else len(supplied)
    options = parser.parse_args(supplied[:split])
    rclpy.init(args=supplied[split:])
    node = client = window = None
    code = 0
    try:
        node = MoveItClient(options)
        frame = options.frame
        node.select_arm(options.arm)
        options.frame = frame
        client = ServoClient(node, options.frame)
        print(client.check())
        if not options.execute:
            print('添加 --execute 打开键盘窗口；按住 Shift 操作，松键/失焦停止。')
        else:
            window = KeyboardWindow(node, client)
            window.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'键盘遥操作失败：{exc}', file=sys.stderr)
        code = 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception as exc:
                print(f'Servo 停止/恢复失败：{exc}；请重启服务确认控制状态。', file=sys.stderr)
                code = 1
        if window is not None:
            window.root.destroy()
        if node is not None:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code

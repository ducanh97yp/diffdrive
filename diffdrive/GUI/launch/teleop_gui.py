#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopGuiNode(Node):
    """Publish Twist tren /cmd_vel, thay the cho teleop_twist_keyboard."""

    def __init__(self):
        super().__init__('teleop_gui')

        self.linear_speed = self.declare_parameter('linear_speed', 0.3).value
        self.angular_speed = self.declare_parameter('angular_speed', 1.0).value

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Lenh hien tai duoc phat lai dinh ky de robot khong dung do timeout
        # cua controller khi khong co nut nao duoc giu.
        self._current_twist = Twist()
        self.timer = self.create_timer(0.1, self._publish_current)

    def _publish_current(self):
        self.publisher.publish(self._current_twist)

    def set_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x * self.linear_speed
        twist.angular.z = angular_z * self.angular_speed
        self._current_twist = twist

    def stop(self):
        self._current_twist = Twist()


class TeleopGuiApp:
    def __init__(self, node: TeleopGuiNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title('Robot Teleop')
        self.root.resizable(False, False)

        pad = {'width': 8, 'height': 3}

        frame = tk.Frame(self.root, padx=10, pady=10)
        frame.grid(row=0, column=0)

        btn_up = tk.Button(frame, text='↑', **pad)
        btn_up.grid(row=0, column=1)
        btn_left = tk.Button(frame, text='←', **pad)
        btn_left.grid(row=1, column=0)
        btn_stop = tk.Button(frame, text='STOP', **pad, fg='red')
        btn_stop.grid(row=1, column=1)
        btn_right = tk.Button(frame, text='→', **pad)
        btn_right.grid(row=1, column=2)
        btn_down = tk.Button(frame, text='↓', **pad)
        btn_down.grid(row=2, column=1)

        self._bind_hold(btn_up, lambda: self.node.set_twist(1.0, 0.0))
        self._bind_hold(btn_down, lambda: self.node.set_twist(-1.0, 0.0))
        self._bind_hold(btn_left, lambda: self.node.set_twist(0.0, 1.0))
        self._bind_hold(btn_right, lambda: self.node.set_twist(0.0, -1.0))
        btn_stop.config(command=self.node.stop)

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _bind_hold(self, button, on_press):
        # Chi phat lenh khi nut dang duoc giu (nhu phim mui ten), tha ra la dung.
        button.bind('<ButtonPress-1>', lambda _event: on_press())
        button.bind('<ButtonRelease-1>', lambda _event: self.node.stop())

    def _on_close(self):
        self.node.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = TeleopGuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = TeleopGuiApp(node)
    try:
        app.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

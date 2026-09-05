#!/usr/bin/env python3
"""Qt keyboard teleop matching the real F1TENTH Joy mapping."""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from PyQt6 import QtCore, QtWidgets


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('sim_keyboard_teleop')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_speed', 4.5)
        self.declare_parameter('max_steer', 0.4788)
        self.declare_parameter('initial_manual_control', False)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        self.manual = bool(self.get_parameter('initial_manual_control').value)
        self.pressed = set()
        self.joy_pub = self.create_publisher(
            Joy, str(self.get_parameter('joy_topic').value), 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_controls)
        self.get_logger().info(
            'Keyboard: M toggles manual/auto; W/S control speed and A/D steer. '
            'Joy mapping: speed axis=1, steer axis=3, manual button=4, auto button=5')

    def set_key(self, key, pressed):
        if pressed:
            self.pressed.add(key)
        else:
            self.pressed.discard(key)
        # Do not wait for the next 20 Hz timer tick. A short tap can otherwise
        # begin and end within one 50 ms period, producing no non-zero Joy axis.
        self.publish_controls()

    def toggle_manual(self):
        self.manual = not self.manual
        self.get_logger().info('Manual control: %s' % self.manual)

    def publish_controls(self):
        longitudinal = float(
            (QtCore.Qt.Key.Key_W in self.pressed)
            - (QtCore.Qt.Key.Key_S in self.pressed))
        steering = float(
            (QtCore.Qt.Key.Key_A in self.pressed)
            - (QtCore.Qt.Key.Key_D in self.pressed))

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 12
        joy.axes[1] = longitudinal
        joy.axes[3] = steering
        joy.buttons[4] = int(self.manual)
        joy.buttons[5] = int(not self.manual)
        self.joy_pub.publish(joy)


class KeyboardWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('F1TENTH simulator keyboard')
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.setMinimumSize(430, 150)
        self.label = QtWidgets.QLabel(self)
        self.label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.label)
        self.refresh_label()

    def refresh_label(self):
        mode = 'MANUAL' if self.node.manual else 'AUTONOMOUS'
        self.label.setText(
            f'Mode: {mode}\n\nM: toggle mode\n'
            'W/S: forward/reverse   A/D: left/right steering\n'
            'Keep this window focused while using the keys.')

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() == QtCore.Qt.Key.Key_M:
            self.node.toggle_manual()
            self.refresh_label()
        elif event.key() in (
                QtCore.Qt.Key.Key_W, QtCore.Qt.Key.Key_S,
                QtCore.Qt.Key.Key_A, QtCore.Qt.Key.Key_D):
            self.node.set_key(event.key(), True)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        self.node.set_key(event.key(), False)

    def focusOutEvent(self, event):
        self.node.pressed.clear()
        super().focusOutEvent(event)

    def closeEvent(self, event):
        self.node.pressed.clear()
        if rclpy.ok():
            self.node.publish_controls()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    app = QtWidgets.QApplication(sys.argv)
    window = KeyboardWindow(node)
    window.show()
    ros_timer = QtCore.QTimer()

    def spin_ros():
        if not rclpy.ok():
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            # ros2 launch/timeout may invalidate the context asynchronously.
            # Quit Qt cleanly in that case, but preserve genuine live errors.
            if rclpy.ok():
                raise
            app.quit()

    ros_timer.timeout.connect(spin_ros)
    ros_timer.start(5)
    try:
        app.exec()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

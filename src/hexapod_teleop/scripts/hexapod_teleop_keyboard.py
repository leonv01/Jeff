#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

HELP_MSG = """
Hexapod Keyboard Teleop (Joy Emulator)
--------------------------------------
Movement (Axes):
        w
   a    s    d      q / e: Turn Left / Turn Right

Gait / Mode Buttons:
   1: Tripod Gait   (button 0)
   2: Ripple Gait   (button 1)
   3: Wave Gait     (button 3)
   t: Toggle Pose   (button 2)
   SPACE: Stand/Sit (button 9)

   CTRL-C to quit
"""

class HexapodKeyboardJoy(Node):
    def __init__(self):
        super().__init__('hexapod_keyboard_joy')
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Current state
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.active_button = None
        self.last_key_time = self.get_clock().now()

        self.get_logger().info("Hexapod Keyboard Joy Node initialized.")

    def update_key(self, key: str):
        self.last_key_time = self.get_clock().now()

        # Movement keys
        if key == 'w':
            self.linear_x = 1.0
        elif key == 's':
            self.linear_x = -1.0
        elif key == 'a':
            self.linear_y = 1.0
        elif key == 'd':
            self.linear_y = -1.0
        elif key == 'q':
            self.angular_z = 1.0
        elif key == 'e':
            self.angular_z = -1.0

        # Buttons (one-shot trigger)
        elif key == '1':
            self.active_button = 0  # tripod
        elif key == '2':
            self.active_button = 1  # ripple
        elif key == '3':
            self.active_button = 3  # wave
        elif key == 't':
            self.active_button = 2  # toggle pose
        elif key == ' ':
            self.active_button = 9  # stand / sit

    def timer_callback(self):
        # Auto-decay movement axes to 0 if no key received for 300ms
        dt = (self.get_clock().now() - self.last_key_time).nanoseconds / 1e9
        if dt > 0.3:
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 12

        joy_msg.axes[1] = self.linear_x
        joy_msg.axes[0] = self.linear_y
        joy_msg.axes[3] = self.angular_z

        if self.active_button is not None:
            joy_msg.buttons[self.active_button] = 1
            self.active_button = None  # Reset button pulse after 1 cycle

        self.joy_pub.publish(joy_msg)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)
    node = HexapodKeyboardJoy()
    print(HELP_MSG)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings)
            if key:
                if key == '\x03':  # Ctrl-C
                    break
                node.update_key(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import termios
import tty


class KeyboardToJoyNode(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)

    def publish_joy_message(self, axes, buttons):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = axes
        joy_msg.buttons = buttons
        self.publisher_.publish(joy_msg)

    def run(self):
        axes = [0.0] * 6
        buttons = [0] * 10

        # Mapping keyboard keys to joystick axes and buttons
        key_mappings = {
            'w': (1, 0),    # Increase axis 1 (forward)
            's': (-1, 0),   # Decrease axis 1 (backward)
            'a': (0, -1),   # Decrease axis 2 (left)
            'd': (0, 1),    # Increase axis 2 (right)
            ' ': (0, 0),    # Stop axes 1 and 2
            '1': 1,         # Press button 1
            '2': 0,         # Release button 1
            '3': 1,         # Press button 2
            '4': 0          # Release button 2
        }

        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            # Read keyboard input
            char = sys.stdin.read(1)
            if char == '\x1b':  # ASCII escape character
                break

            if char in key_mappings:
                value = key_mappings[char]
                if isinstance(value, tuple):
                    axis_index, axis_value = value
                    axes[axis_index] = axis_value
                elif isinstance(value, int):
                    button_index = value
                    buttons[button_index] = 1

            self.publish_joy_message(axes, buttons)
            self.get_logger().info("Published joystick message")

            rclpy.spin_once(self)

        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoyNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Change this to whatever message type your arm uses!


class MinimecGamepad(Node):
    def __init__(self):
        super().__init__('minimec_gamepad')

        # Subscribers
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm_command', 10)
        self.claw_pub = self.create_publisher(String, '/claw_command', 10)

        # Tuning speeds
        self.MAX_SPEED = 0.5  # meters per second
        self.MAX_TURN = 1.0  # radians per second

        self.get_logger().info("🎮 Gamepad Teleop Ready! Waiting for /joy messages...")

    def joy_cb(self, msg):
        # ------------------------------------------------
        # 1. DRIVING (Thumbsticks)
        # Standard Xbox mapping:
        # msg.axes[1] is Left Stick Up/Down (Forward/Back)
        # msg.axes[3] is Right Stick Left/Right (Turning)
        # ------------------------------------------------
        twist = Twist()
        twist.linear.x = msg.axes[1] * self.MAX_SPEED
        twist.angular.z = msg.axes[3] * self.MAX_TURN
        self.cmd_vel_pub.publish(twist)

        # ------------------------------------------------
        # 2. MECHANISMS (Face Buttons)
        # msg.buttons[0] = A (Xbox) / Cross (PS4)
        # msg.buttons[1] = B (Xbox) / Circle (PS4)
        # msg.buttons[2] = X (Xbox) / Square (PS4)
        # msg.buttons[3] = Y (Xbox) / Triangle (PS4)
        # ------------------------------------------------

        # Example: Press 'A' to open claw, 'B' to close
        if msg.buttons[0] == 1:
            self.claw_pub.publish(String(data="OPEN"))
        elif msg.buttons[1] == 1:
            self.claw_pub.publish(String(data="CLOSE"))

        # Example: Press 'Y' to raise arm, 'X' to lower
        if msg.buttons[3] == 1:
            self.arm_pub.publish(String(data="RAISE"))
        elif msg.buttons[2] == 1:
            self.arm_pub.publish(String(data="LOWER"))

        # Example: Safety Stop (Right Bumper)
        if msg.buttons[5] == 1:
            self.get_logger().warn("E-STOP BUTTON PRESSED!")
            stop_twist = Twist()  # Sends all zeros
            self.cmd_vel_pub.publish(stop_twist)


def main(args=None):
    rclpy.init(args=args)
    node = MinimecGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
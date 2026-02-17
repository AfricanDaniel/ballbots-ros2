import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
import time
import math


class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')

        self.subscription = self.create_subscription(
            Point,
            '/tennis_ball_position',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.signal_pub = self.create_publisher(Bool, '/ball_ready', 10)

        # --- TUNING PARAMETERS ---
        self.target_dist = 0.8
        self.kP_dist = 0.5
        self.kP_turn = 2.0

        self.max_speed = 0.3
        self.max_turn = 1.0
        self.dist_tolerance = 0.05
        self.center_tolerance = 0.05

        self.last_msg_time = time.time()
        self.timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info("Ball Chaser Node Started.")

    def listener_callback(self, msg):
        self.last_msg_time = time.time()
        cmd = Twist()

        # --- FIX 1: Filter Invalid Data ---
        # If z is NaN or exactly 0, the camera sees nothing.
        if math.isnan(msg.z) or msg.z == 0.0:
            # Stop and do nothing
            self.publisher_.publish(cmd)
            return

        # --- Distance Control ---
        distance_error = msg.z - self.target_dist

        if distance_error > self.dist_tolerance:
            cmd.linear.x = self.kP_dist * distance_error
        elif distance_error < -self.dist_tolerance:
            cmd.linear.x = self.kP_dist * distance_error
        else:
            # Inside the target zone
            cmd.linear.x = 0.0

            # --- FIX 2: Continuous Signaling ---
            # Only signal if we are also centered
            if abs(msg.x) < self.center_tolerance:
                msg_signal = Bool()
                msg_signal.data = True
                self.signal_pub.publish(msg_signal)
                # We removed "self.signal_sent" check so it repeats!

        # --- Orientation Control ---
        if abs(msg.x) > self.center_tolerance:
            cmd.angular.z = -1.0 * self.kP_turn * msg.x
        else:
            cmd.angular.z = 0.0

        # --- Safety Clamping ---
        cmd.linear.x = max(min(cmd.linear.x, self.max_speed), -self.max_speed)
        cmd.angular.z = max(min(cmd.angular.z, self.max_turn), -self.max_turn)

        self.publisher_.publish(cmd)

    def watchdog_callback(self):
        if time.time() - self.last_msg_time > 1.0:
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
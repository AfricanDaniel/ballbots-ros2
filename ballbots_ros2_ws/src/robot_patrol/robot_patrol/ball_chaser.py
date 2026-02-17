#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool
import math


class BallChaser(Node):

    def __init__(self):
        super().__init__('ball_chaser')

        # =====================================================
        # DECLARE PARAMETERS (Loaded From robot_config.yaml)
        # =====================================================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('arm_raised_deg', 252.0),
                ('arm_lowered_deg', 177.0),
                ('target_dist', 0.35),
                ('tilt_start_dist', 0.8),
            ]
        )

        # =====================================================
        # LOAD PARAMETERS
        # =====================================================
        self.ARM_HORIZONTAL = self.get_parameter('arm_raised_deg').value
        self.ARM_MAX_TILT = self.get_parameter('arm_lowered_deg').value

        self.target_dist = self.get_parameter('target_dist').value
        self.tilt_start_dist = self.get_parameter('tilt_start_dist').value

        # =====================================================
        # SUBSCRIBERS / PUBLISHERS
        # =====================================================
        self.sub_cam = self.create_subscription(
            Point,
            '/tennis_ball_position',
            self.listener_callback,
            10
        )

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_arm = self.create_publisher(Float32, '/arm_command', 10)
        self.pub_signal = self.create_publisher(Bool, '/ball_ready', 10)

        # =====================================================
        # SERVICES
        # =====================================================
        self.srv_start = self.create_service(
            SetBool,
            'start_chasing',
            self.handle_start
        )

        self.srv_stop = self.create_service(
            SetBool,
            'stop_chasing',
            self.handle_stop
        )

        # =====================================================
        # STATE
        # =====================================================
        self.is_active = False
        self.arm_angle = self.ARM_HORIZONTAL

        self.get_logger().info(
            f"Ball Chaser Ready | Horizontal: {self.ARM_HORIZONTAL} | "
            f"Max Tilt: {self.ARM_MAX_TILT}"
        )

    # =====================================================
    # SERVICE HANDLERS
    # =====================================================
    def handle_start(self, request, response):

        self.is_active = request.data

        if self.is_active:
            self.publish_arm_angle(self.ARM_HORIZONTAL)
            msg = "Chasing Started!"
        else:
            self.publish_arm_angle(self.ARM_HORIZONTAL)
            self.stop_robot()
            msg = "Chasing Stopped."

        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def handle_stop(self, request, response):
        return self.handle_start(request, response)

    # =====================================================
    # ARM COMMAND PUBLISHER
    # =====================================================
    def publish_arm_angle(self, angle):
        self.arm_angle = angle
        msg = Float32()
        msg.data = float(angle)
        self.pub_arm.publish(msg)

    # =====================================================
    # MAIN CONTROL LOOP
    # =====================================================
    def listener_callback(self, msg):

        if not self.is_active:
            return

        cmd = Twist()

        # -------------------------------------
        # Sanity Check
        # -------------------------------------
        if math.isnan(msg.z) or msg.z <= 0.0:
            self.stop_robot()
            return

        # -------------------------------------
        # Real Distance Compensation
        # -------------------------------------
        pitch_deg = self.ARM_HORIZONTAL - self.arm_angle
        pitch_rad = math.radians(pitch_deg)

        real_dist = msg.z * math.cos(pitch_rad)

        # -------------------------------------
        # Tilt Logic
        # -------------------------------------
        if real_dist < self.tilt_start_dist:

            ratio = (
                (real_dist - self.target_dist) /
                (self.tilt_start_dist - self.target_dist)
            )

            ratio = max(0.0, min(1.0, ratio))

            # Interpolate between horizontal and max tilt
            new_angle = self.ARM_MAX_TILT + (
                ratio * (self.ARM_HORIZONTAL - self.ARM_MAX_TILT)
            )

            self.publish_arm_angle(new_angle)

        else:
            self.publish_arm_angle(self.ARM_HORIZONTAL)

        # -------------------------------------
        # Linear Motion
        # -------------------------------------
        err_dist = real_dist - self.target_dist

        # Define distance tolerance
        dist_tol = 0.05
        center_tol = 0.05

        if abs(err_dist) > dist_tol:
            # Still too far → move forward/back
            cmd.linear.x = 0.5 * err_dist

            # Allow strafing while approaching
            cmd.linear.y = -1.0 * msg.x

        else:
            # ✅ We are within target distance → FULL STOP
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            # Only signal grab if centered
            if abs(msg.x) < center_tol:
                self.pub_signal.publish(Bool(data=True))

        # # -------------------------------------
        # # Angular Motion
        # # -------------------------------------
        # cmd.angular.z = -2.0 * msg.x
        #
        # # -------------------------------------
        # # Safety Clamps
        # # -------------------------------------
        # cmd.linear.x = max(-0.3, min(0.3, cmd.linear.x))
        # cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))

        # -------------------------------------
        # Safety Clamps
        # -------------------------------------
        cmd.linear.x = max(-0.3, min(0.3, cmd.linear.x))
        cmd.linear.y = max(-0.3, min(0.3, cmd.linear.y))  # Clamp Strafe speed
        # cmd.angular.z is 0, so no clamp needed really

        self.pub_cmd.publish(cmd)

    # =====================================================
    # STOP ROBOT
    # =====================================================
    def stop_robot(self):
        self.pub_cmd.publish(Twist())


# =====================================================
# MAIN
# =====================================================
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
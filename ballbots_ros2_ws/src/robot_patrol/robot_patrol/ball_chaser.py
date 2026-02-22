#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math
import numpy as np


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
                ('claw_open_deg', 152.0),
                ('claw_closed_deg', 173.0),
                ('target_dist', 0.2),
                ('tilt_start_dist', 0.8),
                ('grab_dist', 0.16),
            ]
        )

        # =====================================================
        # LOAD PARAMETERS
        # =====================================================
        self.ARM_HORIZONTAL = self.get_parameter('arm_raised_deg').value
        self.ARM_MAX_TILT = self.get_parameter('arm_lowered_deg').value
        self.CLAW_OPEN = self.get_parameter('claw_open_deg').value
        self.CLAW_CLOSED = self.get_parameter('claw_closed_deg').value
        self.target_dist = self.get_parameter('target_dist').value
        self.tilt_start_dist = self.get_parameter('tilt_start_dist').value
        self.grab_dist = self.get_parameter('grab_dist').value

        # =====================================================
        # PUBLISHERS & SUBSCRIBERS
        # =====================================================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_claw = self.create_publisher(Float32, '/claw_command', 10)
        self.pub_arm = self.create_publisher(Float32, '/arm_command', 10)

        # Vision Topics
        self.sub_ball_zed = self.create_subscription(Point, '/tennis_ball_position', self.zed_callback, 10)
        self.sub_ball_rs = self.create_subscription(Point, '/tennis_ball_position_close', self.rs_callback, 10)
        self.sub_bottle = self.create_subscription(Point, '/bottle_position', self.bottle_callback, 10)

        # Odometry for returning home
        self.sub_odom = self.create_subscription(Odometry, '/zed/zed_node/odom', self.odom_callback, 10)

        self.srv_start = self.create_service(SetBool, '/start_chasing', self.handle_start)

        # =====================================================
        # MISSION STATE MACHINE
        # =====================================================
        self.is_active = False
        self.state = 'IDLE'  # SEARCHING, CHASING_ZED, CHASING_RS, GRABBING, TURNING_HOME, CHASING_BOTTLE, DROPPING

        self.home_x = None
        self.home_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.home_yaw = None
        self.current_yaw = 0.0

        # Motion & Alignment tracking
        self.is_aligned = False
        self.current_vx = 0.0
        self.MAX_ACCEL = 0.02

        # Grab/Drop tracking
        self.grab_confirm_count = 0
        self.GRAB_CONFIRM_NEEDED = 3
        self.arm_angle = self.ARM_HORIZONTAL
        self._active_action = None  # 'GRAB' or 'DROP'

        # Main heartbeat timer for background states (Searching & Turning)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Full Mission Ball Chaser node started!")

    # =====================================================
    # ODOMETRY & MISSION CONTROL
    # =====================================================
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Lock the exact X, Y, and Yaw the moment the mission is started
        if self.home_x is None and self.is_active:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.get_logger().info(f"📍 HOME LOCKED at X:{self.home_x:.2f}, Y:{self.home_y:.2f}")

    def handle_start(self, request, response):
        self.is_active = request.data

        if self.is_active:
            self.publish_arm_angle(self.ARM_HORIZONTAL)
            self.publish_claw(self.CLAW_OPEN)
            self.home_yaw = None  # Forces Odom to record a fresh home direction
            self.state = 'SEARCHING'
            self.get_logger().info("🚀 MISSION STARTED: Searching for balls!")
            msg = "Mission Started!"
        else:
            self.state = 'IDLE'
            self.stop_robot()
            msg = "Mission Stopped!"

        response.success = True
        response.message = msg
        return response
    ###
    def control_loop(self):
        """Runs at 20Hz to control states that don't depend on camera callbacks"""
        if not self.is_active:
            return

        cmd = Twist()

        # STATE: SEARCHING
        if self.state == 'SEARCHING':
            cmd.angular.z = 0.3  # Spin slowly left
            self.pub_cmd.publish(cmd)

        # STATE: TURNING HOME
        elif self.state == 'TURNING_HOME':
            if self.home_x is None or self.home_y is None:
                self.state = 'SEARCHING_BOTTLE'  # Failsafe
                return

            # Calculate angle pointing back to the starting (X,Y) coordinate
            angle_to_home = math.atan2(self.home_y - self.current_y, self.home_x - self.current_x)
            diff = (angle_to_home - self.current_yaw + math.pi) % (2 * math.pi) - math.pi

            # If we are facing home, stop and look for the bottle
            if abs(diff) < 0.15:
                self.stop_robot()
                self.is_aligned = False
                self.state = 'SEARCHING_BOTTLE'
                self.get_logger().info("↩️ Pointing Home! Sweeping for bottle...")
            else:
                # Proportional turn back to home
                cmd.angular.z = max(-0.4, min(0.4, 1.0 * diff))
                self.pub_cmd.publish(cmd)

        # STATE: SEARCHING BOTTLE (Fallback if not immediately seen)
        elif self.state == 'SEARCHING_BOTTLE':
            cmd.angular.z = 0.3  # Spin slowly left until the camera spots it
            self.pub_cmd.publish(cmd)

    # =====================================================
    # ZED CALLBACK (Far Range Ball)
    # =====================================================
    def zed_callback(self, msg):
        if not self.is_active: return

        # If searching and we see a ball, switch to chase mode!
        if self.state == 'SEARCHING':
            self.get_logger().info("🎾 BALL SPOTTED! Chasing...")
            self.state = 'CHASING_ZED'
            self.is_aligned = False

        if self.state != 'CHASING_ZED': return

        # Handoff to RealSense when close
        if msg.z < self.tilt_start_dist:
            self.get_logger().info(f"Handoff to RealSense at {msg.z:.2f}m")
            self.state = 'CHASING_RS'
            self.stop_robot()
            self.is_aligned = False
            self.grab_confirm_count = 0
            return

        # STATE MACHINE: ALIGN THEN DRIVE
        target_vx = 0.0
        target_wz = 0.0

        if not self.is_aligned:
            target_wz = max(-0.4, min(0.4, -1.0 * msg.x))
            if abs(msg.x) < 0.05:
                self.is_aligned = True
        else:
            target_vx = max(-0.3, min(0.3, 0.5 * (msg.z - self.target_dist)))
            target_wz = max(-0.15, min(0.15, -0.5 * msg.x))

        self.current_vx += max(-self.MAX_ACCEL, min(self.MAX_ACCEL, target_vx - self.current_vx))

        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.angular.z = target_wz
        self.pub_cmd.publish(cmd)

    # =====================================================
    # REALSENSE CALLBACK (Close Range Ball)
    # =====================================================
    def rs_callback(self, msg):
        if not self.is_active or self.state != 'CHASING_RS': return
        if math.isnan(msg.z) or msg.z <= 0.0: return

        # Physical Claw Offset (Adjust this to center ball in claw!)
        claw_offset_x = 0.01
        err_x = msg.x - claw_offset_x

        # GRAB TRIGGER
        if msg.z <= self.grab_dist and abs(err_x) < 0.06:
            self.grab_confirm_count += 1
            if self.grab_confirm_count >= self.GRAB_CONFIRM_NEEDED:
                self.execute_grab()
            else:
                self.stop_robot()
            return

        # STATE MACHINE: ALIGN THEN DRIVE
        target_vx = 0.0
        target_wz = 0.0

        if not self.is_aligned:
            target_wz = max(-0.25, min(0.25, -0.8 * err_x))
            if abs(err_x) < 0.03:
                self.is_aligned = True
        else:
            target_vx = max(-0.15, min(0.15, 0.4 * msg.z))
            target_wz = max(-0.1, min(0.1, -0.5 * err_x))

        self.current_vx += max(-self.MAX_ACCEL, min(self.MAX_ACCEL, target_vx - self.current_vx))

        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.angular.z = target_wz
        self.pub_cmd.publish(cmd)

    # =====================================================
    # BOTTLE CALLBACK (Dropping off)
    # =====================================================
    def bottle_callback(self, msg):
        if not self.is_active: return
        if math.isnan(msg.z) or msg.z <= 0.0: return

        # ONLY interrupt if we have finished our Odometry turn and are actively searching!
        if self.state == 'SEARCHING_BOTTLE':
            self.get_logger().info("🧴 PINK BOTTLE SPOTTED! Visual Servoing taking over...")
            self.state = 'CHASING_BOTTLE'
            self.is_aligned = False

        if self.state != 'CHASING_BOTTLE': return

        # We stop a bit further from the bottle (20cm) so we don't crash into it
        drop_dist = 0.40
        err_dist = msg.z - drop_dist

        # DROP TRIGGER
        if msg.z <= drop_dist and abs(msg.x) < 0.05:
            self.execute_drop()
            return

        # STATE MACHINE: ALIGN THEN DRIVE
        target_vx = 0.0
        target_wz = 0.0

        if not self.is_aligned:
            target_wz = max(-0.3, min(0.3, -0.8 * msg.x))
            if abs(msg.x) < 0.05:
                self.is_aligned = True
        else:
            target_vx = max(-0.2, min(0.2, 0.5 * err_dist))
            target_wz = max(-0.1, min(0.1, -0.5 * msg.x))

        self.current_vx += max(-self.MAX_ACCEL, min(self.MAX_ACCEL, target_vx - self.current_vx))

        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.angular.z = target_wz
        self.pub_cmd.publish(cmd)

    # =====================================================
    # ARM KINEMATICS (GRAB & DROP)
    # =====================================================
    def execute_grab(self):
        self.get_logger().info("======================================")
        self.get_logger().info("🎾 BALL SECURED — Executing Grab...")
        self.state = 'GRABBING'
        self.stop_robot()
        self.publish_claw(self.CLAW_OPEN)
        self._target_arm_angle = self.ARM_MAX_TILT
        self._arm_step_dir = -1.0
        self._active_action = 'GRAB'
        self._arm_sweep_timer = self.create_timer(0.02, self._sweep_arm_cb)

    def execute_drop(self):
        self.get_logger().info("======================================")
        self.get_logger().info("🧴 BOTTLE REACHED — Executing Drop...")
        self.state = 'DROPPING'
        self.stop_robot()
        self._target_arm_angle = self.ARM_MAX_TILT
        self._arm_step_dir = -1.0
        self._active_action = 'DROP'
        self._arm_sweep_timer = self.create_timer(0.02, self._sweep_arm_cb)

    def _sweep_arm_cb(self):
        """Universal smooth arm sweep for both grabbing and dropping"""
        step_size = 1.0
        self.arm_angle += (step_size * self._arm_step_dir)

        if (self._arm_step_dir < 0 and self.arm_angle <= self._target_arm_angle) or \
                (self._arm_step_dir > 0 and self.arm_angle >= self._target_arm_angle):

            self.arm_angle = self._target_arm_angle
            self.publish_arm_angle(self.arm_angle)
            self._arm_sweep_timer.cancel()

            if self.arm_angle == self.ARM_MAX_TILT:
                # Reached the floor. What are we doing?
                if self._active_action == 'GRAB':
                    self._action_timer = self.create_timer(0.5, self._close_claw_cb)
                else:
                    self._action_timer = self.create_timer(0.5, self._open_claw_drop_cb)
            else:
                # Reached the top. Sequence complete!
                if self._active_action == 'GRAB':
                    self.get_logger().info("✅ Grab complete. Turning back home.")
                    self.state = 'TURNING_HOME'
                else:
                    self.get_logger().info("✅ Drop complete. Searching for next ball!")
                    self.state = 'SEARCHING'
        else:
            self.publish_arm_angle(self.arm_angle)

    def _close_claw_cb(self):
        self._action_timer.cancel()
        self.publish_claw(self.CLAW_CLOSED)
        self._action_timer = self.create_timer(1.0, self._start_raise_arm_cb)

    def _open_claw_drop_cb(self):
        self._action_timer.cancel()
        self.publish_claw(self.CLAW_OPEN)
        self._action_timer = self.create_timer(1.0, self._start_raise_arm_cb)

    def _start_raise_arm_cb(self):
        self._action_timer.cancel()
        self._target_arm_angle = self.ARM_HORIZONTAL
        self._arm_step_dir = 1.0
        self._arm_sweep_timer = self.create_timer(0.02, self._sweep_arm_cb)

    # =====================================================
    # UTILS
    # =====================================================
    def stop_robot(self):
        self.current_vx = 0.0
        self.pub_cmd.publish(Twist())

    def publish_arm_angle(self, angle_deg):
        self.arm_angle = angle_deg
        msg = Float32()
        msg.data = float(angle_deg)
        self.pub_arm.publish(msg)

    def publish_claw(self, angle_deg):
        msg = Float32()
        msg.data = float(angle_deg)
        self.pub_claw.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
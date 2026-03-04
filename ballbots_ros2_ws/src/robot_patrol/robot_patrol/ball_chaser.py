#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

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

        # TF

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        self._search_spinning = False
        self._search_timer = None

        self.home_yaw = None
        self.current_yaw = 0.0

        self._bottle_confirm_count = 0
        self.BOTTLE_CONFIRM_NEEDED = 2  # must see bottle 5 frames in a row

        # Motion & Alignment tracking
        self.is_aligned = False
        self.current_vx = 0.0
        self.MAX_ACCEL = 0.02

        # Grab/Drop tracking
        self.grab_confirm_count = 0
        self.GRAB_CONFIRM_NEEDED = 3
        self.arm_angle = self.ARM_HORIZONTAL
        self._active_action = None  # 'GRAB' or 'DROP'
        # Drop zone exclusion
        self._drop_position = None
        self.DROP_EXCLUSION_RADIUS = 0.2

        # Main heartbeat timer for background states (Searching & Turning)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        #reset timer if we have not seen it in a while
        self.create_timer(0.1, self._bottle_confirm_reset_cb)

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
                self.state = 'SEARCHING_BOTTLE'
                return

            dist_from_home = math.sqrt(
                (self.current_x - self.home_x) ** 2 +
                (self.current_y - self.home_y) ** 2
            )

            # Already at home — skip turning, go straight to bottle search
            if dist_from_home < 0.3:
                self.get_logger().info("📍 Already near home — skipping turn")
                self.state = 'SEARCHING_BOTTLE'
                return

            # Calculate angle pointing back to home coordinates
            angle_to_home = math.atan2(self.home_y - self.current_y, self.home_x - self.current_x)
            diff = (angle_to_home - self.current_yaw + math.pi) % (2 * math.pi) - math.pi

            if abs(diff) < 0.15:
                self.stop_robot()
                self.is_aligned = False
                self._search_spinning = False
                if self._search_timer is not None:
                    self._search_timer.cancel()
                    self._search_timer = None

                # Try to face bottle TF before spinning blindly
                try:
                    tf = self.tf_buffer.lookup_transform(
                        'base_link', 'neon_bottle',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    bx = tf.transform.translation.x
                    bz = tf.transform.translation.z
                    angle_to_bottle = math.atan2(bx, bz)
                    self.get_logger().info(
                        f"↩️ Home! Bottle TF at angle={math.degrees(angle_to_bottle):.1f}° — turning to face it")
                    self._bottle_target_angle = self.current_yaw + angle_to_bottle
                    self.state = 'TURNING_TO_BOTTLE'
                except Exception:
                    self.get_logger().info("↩️ Home! No bottle TF — sweeping...")
                    self.state = 'SEARCHING_BOTTLE'
            else:
                # Still turning toward home
                cmd.angular.z = max(-0.4, min(0.4, 1.0 * diff))
                self.pub_cmd.publish(cmd)

        # STATE: TURNING TO BOTTLE (using last known TF)
        elif self.state == 'TURNING_TO_BOTTLE':
            diff_bottle = (self._bottle_target_angle - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(diff_bottle) < 0.15:
                self.stop_robot()
                self.state = 'SEARCHING_BOTTLE'
                self.get_logger().info("🎯 Facing bottle direction — confirming visually...")
            else:
                cmd.angular.z = max(-0.3, min(0.3, 1.0 * diff_bottle))
                self.pub_cmd.publish(cmd)

        # STATE: SEARCHING BOTTLE (Fallback if not immediately seen)
        elif self.state == 'SEARCHING_BOTTLE':
            if not self._search_spinning:
                cmd.angular.z = 0.15
                self.pub_cmd.publish(cmd)
                self._search_spinning = True
                self._search_timer = self.create_timer(0.5, self._search_pause_cb)

    # =====================================================
    # ZED CALLBACK (Far Range Ball)
    # =====================================================
    def zed_callback(self, msg):
        if not self.is_active: return

        if self.state == 'SEARCHING':
            # Ignore ball if too close to drop zone
            if self._drop_position is not None:
                dist_from_drop = math.sqrt(
                    (self.current_x - self._drop_position[0]) ** 2 +
                    (self.current_y - self._drop_position[1]) ** 2
                )
                if dist_from_drop < self.DROP_EXCLUSION_RADIUS:
                    return
            self.stop_robot()
            self.get_logger().info("🎾 BALL SPOTTED! Chasing...")
            self.state = 'CHASING_ZED'
            self.is_aligned = False
            return

        if self.state != 'CHASING_ZED': return

        # ← ADD: grab trigger before anything else
        if msg.z <= self.grab_dist and abs(msg.x) < 0.06:
            self.grab_confirm_count += 1
            if self.grab_confirm_count >= self.GRAB_CONFIRM_NEEDED:
                self.execute_grab()
            else:
                self.stop_robot()
            return

        # Align then drive
        target_vx = 0.0
        target_wz = 0.0

        if not self.is_aligned:
            target_wz = max(-0.25, min(0.25, -0.6 * msg.x))
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

        if self.state == 'SEARCHING_BOTTLE':
            self._bottle_confirm_count += 1
            self.get_logger().info(f"🧴 Bottle seen ({self._bottle_confirm_count}/3)")
            if self._bottle_confirm_count < 3:  # was 5, reduce to 3
                # Stop spinning immediately even before confirmed
                self.stop_robot()  # ← ADD THIS — kill spin on first sight
                self._search_spinning = False
                if self._search_timer is not None:
                    self._search_timer.cancel()
                    self._search_timer = None
                return
            self.get_logger().info("🧴 PINK BOTTLE CONFIRMED!")
            self._bottle_confirm_count = 0
            self.state = 'CHASING_BOTTLE'
            return

        if self.state != 'CHASING_BOTTLE': return

        drop_dist = 0.40

        # DROP TRIGGER
        if msg.z <= drop_dist and abs(msg.x) < 0.08:
            if self.state == 'CHASING_BOTTLE':  # ← only trigger once
                self.execute_drop()
            return

        # Simple proportional control — same as ball chase, no acceleration ramp
        err_dist = msg.z - drop_dist
        target_vx = max(-0.25, min(0.25, 0.5 * err_dist))  # was 0.15 max, now 0.25
        target_wz = max(-0.3, min(0.3, -0.6 * msg.x))  # was 0.3 max, now 0.6 gain

        # Only stop forward motion if very off-center
        if abs(msg.x) > 0.25:  # was 0.20
            target_vx = 0.0

        # Skip MAX_ACCEL ramp — publish directly like zed_callback does
        cmd = Twist()
        cmd.linear.x = target_vx
        cmd.angular.z = target_wz
        self.pub_cmd.publish(cmd)

    def _bottle_confirm_reset_cb(self):
        pass
        # If no bottle message arrived recently, reset counter
        # This is handled implicitly — bottle_callback only increments when called
        # So if bottle disappears for a frame, we need to decay the counter
        # if self.state == 'SEARCHING_BOTTLE':
        #     if self._bottle_confirm_count > 0:
        #         self._bottle_confirm_count = max(0, self._bottle_confirm_count - 1)

    def _search_pause_cb(self):
        """Called after spin burst — stop and wait for sharp frame"""
        if self._search_timer is not None:
            self._search_timer.cancel()
            self._search_timer = None
        self.stop_robot()
        self._search_timer = self.create_timer(0.3, self._search_resume_cb)

    def _search_resume_cb(self):
        """Called after pause — ready for next spin burst"""
        if self._search_timer is not None:
            self._search_timer.cancel()
            self._search_timer = None
        self._search_spinning = False

    # =====================================================
    # ARM KINEMATICS (GRAB & DROP)
    # =====================================================
    def execute_grab(self):
        self.get_logger().info("======================================")
        self.get_logger().info("🎾 BALL SECURED — Executing Grab...")
        self.state = 'GRABBING'
        self.stop_robot()
        self.current_vx = 0.0  # ← force reset acceleration
        self.is_aligned = False  # ← reset alignment for next chase
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
                    self._drop_position = (self.current_x, self.current_y)
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
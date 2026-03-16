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
from collections import deque
from std_srvs.srv import Trigger


class BallChaser(Node):

    def __init__(self):
        super().__init__('ball_chaser')

        # =====================================================
        # PARAMETERS
        # =====================================================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('arm_raised_deg',   252.0),
                ('arm_lowered_deg',  177.0),
                ('claw_open_deg',    152.0),
                ('claw_closed_deg',  173.0),
                ('target_dist',      0.10),
                ('tilt_start_dist',  0.01),
                ('grab_dist',        0.16),
                ('use_zed',          False),
            ]
        )
        self.ARM_HORIZONTAL  = self.get_parameter('arm_raised_deg').value
        self.ARM_MAX_TILT    = self.get_parameter('arm_lowered_deg').value
        self.CLAW_OPEN       = self.get_parameter('claw_open_deg').value
        self.CLAW_CLOSED     = self.get_parameter('claw_closed_deg').value
        self.target_dist     = self.get_parameter('target_dist').value
        self.tilt_start_dist = self.get_parameter('tilt_start_dist').value
        self.grab_dist       = self.get_parameter('grab_dist').value
        self.use_zed         = self.get_parameter('use_zed').value

        # =====================================================
        # MISSION TUNING
        # =====================================================
        self.COURT_TURN_DEG        = 40.0   # degrees LEFT at startup to face court
        self.RETURN_TURN_DEG       = 60.0   # degrees to turn after drop (USE_POLE_DETECTION=False)

        self.BALL_STILL_FRAMES       = 4     # consecutive still frames needed (4 @ 2Hz ≈ 2s)
        self.BALL_STILL_THRESHOLD_X  = 0.10  # max x change between frames (metres)
        self.BALL_STILL_THRESHOLD_Z  = 0.50  # max z change between frames (metres, <4m only)
        self.DROP_EXCLUSION_RADIUS   = 0.2
        self.BOTTLE_TURN_TIMEOUT     = 370.0

        # =====================================================
        # COURT POLE DETECTION TOGGLE
        # ── True  = rotate left until pole seen, then nudge right POLE_ALIGN_DEG
        # ── False = rotate fixed RETURN_TURN_DEG (hardcoded, good for testing)
        # =====================================================
        self.USE_POLE_DETECTION = True

        self.POLE_CONFIRM_NEEDED = 2      # consecutive frames before trusting pole
        self.POLE_ALIGN_DEG      = 0.0   # degrees to nudge RIGHT after seeing pole
                                           # tune so robot ends up facing the court

        # =====================================================
        # PUBLISHERS & SUBSCRIBERS
        # =====================================================
        self.pub_cmd  = self.create_publisher(Twist,   '/cmd_vel',      10)
        self.pub_claw = self.create_publisher(Float32, '/claw_command',  10)
        self.pub_arm  = self.create_publisher(Float32, '/arm_command',   10)

        self.sub_ball_zed = self.create_subscription(Point,    '/tennis_ball_position',       self.zed_callback,    10)
        self.sub_ball_rs  = self.create_subscription(Point,    '/tennis_ball_position_close', self.rs_callback,     10)
        self.sub_bottle   = self.create_subscription(Point,    '/bottle_position',            self.bottle_callback, 10)
        self.sub_odom     = self.create_subscription(Odometry, '/zed/zed_node/odom',          self.odom_callback,   10)
        self.sub_pole     = self.create_subscription(Point,    '/court_pole_position',        self.pole_callback,   10)

        self.srv_start = self.create_service(SetBool, '/start_chasing', self.handle_start)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # =====================================================
        # STATE MACHINE
        # =====================================================
        # Flow:
        #  IDLE
        #  → INIT_SCAN_BOTTLE       : wait 2s, try to lock bottle TF
        #  → INIT_TURN_TO_COURT     : rotate COURT_TURN_DEG left
        #  → WAITING_FOR_BALL       : static wait until ball still for 3s
        #  → CHASING_ZED            : drive toward ball (far)
        #  → CHASING_RS             : drive toward ball (close, optional)
        #  → GRABBING               : arm/claw grab sequence
        #  → TURNING_TO_BOTTLE      : rotate to known bottle angle
        #  → SEARCHING_BOTTLE       : slow sweep scan for bottle
        #  → CHASING_BOTTLE         : drive toward bottle
        #  → DROPPING               : arm/claw drop sequence
        #
        #  [USE_POLE_DETECTION=True]
        #  → SEARCHING_COURT_POLE   : rotate left until pole seen
        #  → ALIGNING_TO_COURT      : nudge right POLE_ALIGN_DEG, then WAITING_FOR_BALL
        #
        #  [USE_POLE_DETECTION=False]
        #  → TURNING_TO_COURT       : rotate fixed RETURN_TURN_DEG, then WAITING_FOR_BALL
        # =====================================================
        self.is_active   = False
        self.state       = 'IDLE'

        # Odometry
        self.home_x      = None
        self.home_y      = None
        self.home_yaw    = None
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        # Yaw targets
        self.court_yaw            = None
        self.bottle_yaw_locked    = None
        self._bottle_target_angle = 0.0
        self._turn_start_yaw      = None
        self._turn_settle_since   = None

        # Ball stillness
        self._ball_history     = deque(maxlen=2)  # only need last 2 frames
        self._ball_still_count = 0               # consecutive still frames

        # Drop zone
        self._drop_position = None

        # Bottle confirmation
        self._bottle_confirm_count = 0
        self.BOTTLE_CONFIRM_NEEDED = 3

        # Pole
        self._pole_confirm_count = 0
        self._pole_align_yaw     = None

        # Search spin
        self._search_spinning = False
        self._search_timer    = None

        # Alignment / velocity
        self.is_aligned       = False
        self._align_start_t   = None   # time we entered not-aligned phase
        self.ALIGN_TIMEOUT    = 4.0    # seconds before forcing forward motion
        self.current_vx       = 0.0
        self.MAX_ACCEL        = 0.02
        self._smooth_x        = None   # EMA of ball x during chasing

        # Grab/drop
        self.grab_confirm_count  = 0
        self.GRAB_CONFIRM_NEEDED = 2
        self.arm_angle           = self.ARM_HORIZONTAL
        self._active_action      = None
        self._arm_sweep_timer    = None
        self._action_timer       = None

        # Init scan
        self._init_scan_timer = None
        self._init_scan_done  = False

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.create_timer(0.1, self._bottle_confirm_reset_cb)

        self.get_logger().info("Full Mission Ball Chaser node started!")
        self.get_logger().info(
            f"  use_zed={self.use_zed}  "
            f"USE_POLE_DETECTION={self.USE_POLE_DETECTION}  "
            f"POLE_ALIGN_DEG={self.POLE_ALIGN_DEG}°  "
            f"RETURN_TURN_DEG={self.RETURN_TURN_DEG}°")

    # =====================================================
    # ODOMETRY
    # =====================================================
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.home_x is None and self.is_active:
            self.home_x   = self.current_x
            self.home_y   = self.current_y
            self.home_yaw = self.current_yaw
            self.get_logger().info(
                f"📍 HOME LOCKED X:{self.home_x:.2f} Y:{self.home_y:.2f} "
                f"Yaw:{math.degrees(self.home_yaw):.1f}°"
            )

    # =====================================================
    # START / STOP
    # =====================================================
    def handle_start(self, request, response):
        self.is_active = request.data

        if self.is_active:
            self.publish_arm_angle(self.ARM_HORIZONTAL)
            self.publish_claw(self.CLAW_OPEN)
            self.home_x = self.home_y = self.home_yaw = None
            self.court_yaw = self.bottle_yaw_locked = None
            self._drop_position = None
            self._ball_history.clear()
            self._ball_still_count = 0
            self.grab_confirm_count = 0
            self._bottle_confirm_count = 0
            self._pole_confirm_count = 0
            self._pole_align_yaw = None
            self._init_scan_done = False
            self._search_spinning = False
            self.current_vx = 0.0
            self._turn_settle_since = None
            self.state = 'INIT_SCAN_BOTTLE'
            self.get_logger().info("🚀 MISSION STARTED — scanning for bottle TF...")
        else:
            self.state = 'IDLE'
            self.stop_robot()

        response.success = True
        response.message = "Mission Started!" if self.is_active else "Mission Stopped!"
        return response

    # =====================================================
    # CONTROL LOOP (20 Hz)
    # =====================================================
    def control_loop(self):
        if not self.is_active:
            return

        cmd = Twist()

        # ─── INIT: wait for bottle TF ───────────────────────────────
        if self.state == 'INIT_SCAN_BOTTLE':
            if not self._init_scan_done and self._init_scan_timer is None:
                self.get_logger().info("🔍 Waiting 2s for bottle TF...")
                self._init_scan_timer = self.create_timer(2.0, self._init_scan_done_cb)

        # ─── INIT: turn left to face court ──────────────────────────
        elif self.state == 'INIT_TURN_TO_COURT':

            if self.USE_POLE_DETECTION:
                # Rotate left until pole seen — pole_callback handles transition
                # but we need a different target state: WAITING_FOR_BALL via ALIGNING_TO_COURT
                if not hasattr(self, '_init_pole_search_started') or not self._init_pole_search_started:
                    self._init_pole_search_started = True
                    self._pole_confirm_count = 0
                    self._pole_align_yaw = None
                    self._turn_settle_since = None
                    # Temporarily redirect pole detection to init flow
                    self._pole_target_state = 'INIT_ALIGN'
                    self.get_logger().info("🔍 Searching for court pole to face court (init)...")

                cmd.angular.z = 0.1  # CCW = left
                self.pub_cmd.publish(cmd)

            else:
                # Hardcoded fixed angle turn
                if self.court_yaw is None and self.home_yaw is not None:
                    self.court_yaw = self._wrap(self.home_yaw + math.radians(self.COURT_TURN_DEG))
                    self._turn_start_yaw = self.current_yaw
                    self.get_logger().info(
                        f"🔄 Turning {self.COURT_TURN_DEG}° left → court_yaw={math.degrees(self.court_yaw):.1f}°"
                    )

                if self.court_yaw is None:
                    return

                diff = self._adiff(self.court_yaw, self.current_yaw)
                if abs(diff) < 0.08:
                    self.stop_robot()
                    self._turn_start_yaw = None
                    self.state = 'WAITING_FOR_BALL'
                    self.get_logger().info("✅ Facing court — waiting for still ball...")
                else:
                    speed = max(0.15, min(0.6, abs(diff) * 1.2))
                    cmd.angular.z = speed
                    self.pub_cmd.publish(cmd)

        # ─── WAIT: watching for a stationary ball ───────────────────
        elif self.state == 'WAITING_FOR_BALL':
            self.stop_robot()

        # ─── SEARCHING (fallback) ────────────────────────────────────
        elif self.state == 'SEARCHING':
            if not self._search_spinning:
                cmd.angular.z = 0.3
                self.pub_cmd.publish(cmd)
                self._search_spinning = True
                self._search_timer = self.create_timer(0.5, self._search_pause_cb)

        # ─── TURNING TO BOTTLE ───────────────────────────────────────
        elif self.state == 'TURNING_TO_BOTTLE':
            if self._turn_start_yaw is None:
                self._turn_start_yaw = self.current_yaw

            traveled = abs(self._adiff(self.current_yaw, self._turn_start_yaw))
            if traveled > math.radians(self.BOTTLE_TURN_TIMEOUT):
                self.get_logger().warn("⚠️  TURNING_TO_BOTTLE timeout — sweeping...")
                self._turn_start_yaw = None
                self.state = 'SEARCHING_BOTTLE'
                return

            diff = self._adiff(self._bottle_target_angle, self.current_yaw)
            if abs(diff) < 0.12:
                self.stop_robot()
                self._turn_start_yaw = None
                self.state = 'SEARCHING_BOTTLE'
                self.get_logger().info("🎯 Facing bottle direction — confirming visually...")
            else:
                speed = max(0.15, min(0.6, abs(diff) * 1.2))
                cmd.angular.z = speed
                self.pub_cmd.publish(cmd)

        # ─── SEARCHING BOTTLE ────────────────────────────────────────
        elif self.state == 'SEARCHING_BOTTLE':
            if not self._search_spinning:
                cmd.angular.z = 0.6
                self.pub_cmd.publish(cmd)
                self._search_spinning = True
                self._search_timer = self.create_timer(0.5, self._search_pause_cb)

        # ─── SEARCHING COURT POLE: rotate left until pole seen ───────
        elif self.state == 'SEARCHING_COURT_POLE':
            # Stop immediately on first pole sighting — pole_callback handles that
            if self._pole_confirm_count == 0:
                cmd.angular.z = 0.3   # CCW = left
                self.pub_cmd.publish(cmd)

        # ─── ALIGNING TO COURT: nudge right after pole seen ──────────
        elif self.state == 'ALIGNING_TO_COURT':
            if self._pole_align_yaw is None:
                self.get_logger().info("✅ Pole confirmed — transitioning to WAITING_FOR_BALL")
                self._pole_align_yaw = self.current_yaw  # dummy value so diff=0 below

            diff = self._adiff(self._pole_align_yaw, self.current_yaw)
            if abs(diff) < 0.08:
                if self._turn_settle_since is None:
                    self._turn_settle_since = self.get_clock().now().nanoseconds / 1e9
                elif (self.get_clock().now().nanoseconds / 1e9 - self._turn_settle_since) > 0.4:
                    self.stop_robot()
                    self._turn_settle_since  = None
                    self._pole_align_yaw     = None
                    self._pole_confirm_count = 0
                    self._ball_history.clear()
                    self._ball_still_count = 0
                    self.state = 'WAITING_FOR_BALL'
                    self.get_logger().info("✅ Aligned to court — waiting for next ball...")
                self.stop_robot()
            else:
                self._turn_settle_since = None
                # Negative = CW = right
                speed = max(0.12, min(0.5, abs(diff) * 1.2))
                cmd.angular.z = -speed
                self.pub_cmd.publish(cmd)

        # ─── TURNING BACK TO COURT (hardcoded, USE_POLE_DETECTION=False) ─
        elif self.state == 'TURNING_TO_COURT':
            if self.court_yaw is None:
                self.state = 'WAITING_FOR_BALL'
                return

            diff = self._adiff(self.court_yaw, self.current_yaw)
            if abs(diff) < 0.15:
                if self._turn_settle_since is None:
                    self._turn_settle_since = self.get_clock().now().nanoseconds / 1e9
                elif (self.get_clock().now().nanoseconds / 1e9 - self._turn_settle_since) > 0.5:
                    self.stop_robot()
                    self._turn_settle_since = None
                    self._turn_start_yaw    = None
                    self._ball_history.clear()
                    self._ball_still_count = 0
                    self.state = 'WAITING_FOR_BALL'
                    self.get_logger().info("✅ Facing court again — waiting for next ball...")
                self.stop_robot()
            else:
                self._turn_settle_since = None
                cmd.angular.z = max(-0.6, min(0.6, 1.5 * diff))
                self.pub_cmd.publish(cmd)

    # =====================================================
    # INIT SCAN CALLBACK
    # =====================================================
    def _init_scan_done_cb(self):
        if self._init_scan_timer is not None:
            self._init_scan_timer.cancel()
            self._init_scan_timer = None
        self._init_scan_done = True

        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'neon_bottle',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            bx = tf.transform.translation.x
            bz = tf.transform.translation.z
            angle = math.atan2(bx, bz)
            self.bottle_yaw_locked = self._wrap(self.current_yaw + angle)
            self.get_logger().info(
                f"🧴 Bottle TF locked! angle={math.degrees(angle):.1f}° "
                f"absolute_yaw={math.degrees(self.bottle_yaw_locked):.1f}°"
            )
        except Exception:
            self.get_logger().warn("⚠️  No bottle TF at startup — will try again after grab")

        self.state = 'INIT_TURN_TO_COURT'

    # =====================================================
    # POLE CALLBACK
    # =====================================================
    def pole_callback(self, msg):
        if not self.is_active: return

        # Handle both init and post-drop pole search
        valid_states = ('SEARCHING_COURT_POLE', 'INIT_TURN_TO_COURT')
        if self.state not in valid_states: return
        if not self.USE_POLE_DETECTION: return

        # Stop rotating immediately on first sighting
        if self._pole_confirm_count == 0:
            self.stop_robot()
            self.get_logger().info("🎾 Pole spotted — stopped! Confirming...")

        self._pole_confirm_count += 1
        self.get_logger().info(
            f"🎾 Court pole seen ({self._pole_confirm_count}/{self.POLE_CONFIRM_NEEDED})")

        if self._pole_confirm_count >= self.POLE_CONFIRM_NEEDED:
            self._pole_align_yaw = None
            self._turn_settle_since = None
            self._init_pole_search_started = False  # reset for next time
            self.state = 'ALIGNING_TO_COURT'
            self.get_logger().info("🎾 Pole confirmed! Nudging right to face court...")

    # =====================================================
    # ZED CALLBACK (ball tracking + chasing)
    # =====================================================
    def zed_callback(self, msg):
        if not self.is_active: return
        if not self.use_zed: return

        if self.state in ('WAITING_FOR_BALL',):
            self._track_ball_stillness(msg)
            return

        if self.state != 'CHASING_ZED': return

        if msg.z < self.tilt_start_dist:
            self.get_logger().info(f"Handoff to RealSense at {msg.z:.2f}m")
            self.state = 'CHASING_RS'
            self.stop_robot()
            self.is_aligned = False
            self._align_start_t = self.get_clock().now().nanoseconds / 1e9
            self.grab_confirm_count = 0
            self._smooth_x = None
            return

        if msg.z <= self.grab_dist and abs(msg.x) < 0.06:
            self.grab_confirm_count += 1
            if self.grab_confirm_count >= self.GRAB_CONFIRM_NEEDED:
                self.execute_grab()
            else:
                self.stop_robot()
            return

        target_vx = 0.0
        target_wz  = 0.0
        if not self.is_aligned:
            target_wz = max(-0.25, min(0.25, -0.6 * msg.x))
            if abs(msg.x) < 0.05:
                self.is_aligned = True
            else:
                # Creep forward while turning — scale speed down by misalignment
                # align_scale: 1.0 at x=0, 0.0 at x=±0.2m
                align_scale = max(0.0, 1.0 - abs(msg.x) / 0.2)
                target_vx = max(0.0, min(0.12, 0.3 * (msg.z - self.target_dist))) * align_scale
        else:
            target_vx = max(-0.3, min(0.3, 0.5 * (msg.z - self.target_dist)))
            target_wz = max(-0.15, min(0.15, -0.5 * msg.x))

        self.current_vx += max(-self.MAX_ACCEL, min(self.MAX_ACCEL, target_vx - self.current_vx))
        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.angular.z = target_wz
        self.pub_cmd.publish(cmd)

    # =====================================================
    # REALSENSE CALLBACK (close range)
    # =====================================================
    def rs_callback(self, msg):
        if not self.is_active: return
        if math.isnan(msg.z) or msg.z <= 0.0: return

        # RS-only mode: also handle ball stillness detection while waiting
        if not self.use_zed and self.state == 'WAITING_FOR_BALL':
            self._track_ball_stillness(msg)
            return

        if self.state != 'CHASING_RS': return

        claw_offset_x = 0.01
        err_x = msg.x - claw_offset_x

        if msg.z <= self.grab_dist and abs(err_x) < 0.10:
            self.grab_confirm_count += 1
            if self.grab_confirm_count >= self.GRAB_CONFIRM_NEEDED:
                self.execute_grab()
            else:
                self.stop_robot()
            return

        target_vx = 0.0
        target_wz  = 0.0
        if not self.is_aligned:
            # Stop and rotate only — no forward motion until roughly aligned
            target_wz = max(-0.2, min(0.2, -0.5 * err_x))
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self._align_start_t if self._align_start_t else 0.0
            if abs(err_x) < 0.10 or elapsed > self.ALIGN_TIMEOUT:
                if elapsed > self.ALIGN_TIMEOUT:
                    self.get_logger().warn(f"⚠️  Align timeout ({elapsed:.1f}s) — forcing forward")
                self.is_aligned = True
        else:
            if msg.z > self.tilt_start_dist:
                # Far: move fast like bottle approach
                target_vx = max(0.0, min(0.4, 0.6 * (msg.z - self.target_dist)))
            else:
                # Close: slow proportional to avoid running over ball
                target_vx = max(0.0, min(0.15, 0.4 * msg.z))
            target_wz = max(-0.1,  min(0.1,  -0.5 * err_x))

        self.current_vx += max(-self.MAX_ACCEL, min(self.MAX_ACCEL, target_vx - self.current_vx))
        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.angular.z = target_wz
        self.pub_cmd.publish(cmd)

    # =====================================================
    # BALL STILLNESS DETECTION
    # =====================================================
    def _track_ball_stillness(self, msg):
        # Ignore very close detections — likely dropped ball or noise
        # ZED blind spot is <0.6m so 0.6m is a safe lower bound; RS-only can see from ~0.3m
        min_z = 0.6 if self.use_zed else 0.3
        if msg.z < min_z:
            return

        if self._drop_position is not None:
            ball_world_x = self.current_x + msg.z * math.cos(self.current_yaw) \
                           - msg.x * math.sin(self.current_yaw)
            ball_world_y = self.current_y + msg.z * math.sin(self.current_yaw) \
                           + msg.x * math.cos(self.current_yaw)
            dx = ball_world_x - self._drop_position[0]
            dy = ball_world_y - self._drop_position[1]
            if math.sqrt(dx * dx + dy * dy) < self.DROP_EXCLUSION_RADIUS:
                return

        # Compare this frame against the previous one
        if len(self._ball_history) > 0:
            _, prev_x, prev_z = self._ball_history[-1]
            dx = abs(msg.x - prev_x)
            dz = abs(msg.z - prev_z)

            # Beyond 4m RealSense z is too noisy — only check x
            z_ok = True if msg.z > 4.0 else dz < self.BALL_STILL_THRESHOLD_Z
            x_ok = dx < self.BALL_STILL_THRESHOLD_X

            if x_ok and z_ok:
                self._ball_still_count += 1
            else:
                self._ball_still_count = max(0, self._ball_still_count - 1)
                self.get_logger().info(
                    f"🏃 Noisy frame (dx={dx:.3f}m dz={dz:.3f}m) — count={self._ball_still_count}/{self.BALL_STILL_FRAMES}")

            self.get_logger().info(
                f"⏳ Still check: dx={dx:.3f}m dz={dz:.3f}m "
                f"count={self._ball_still_count}/{self.BALL_STILL_FRAMES}",
                throttle_duration_sec=1.0)

            if self._ball_still_count >= self.BALL_STILL_FRAMES:
                next_state = 'CHASING_ZED' if self.use_zed else 'CHASING_RS'
                self.get_logger().info(
                    f"🎾 Ball still for {self.BALL_STILL_FRAMES} frames — CHASING! ({next_state})")
                self.stop_robot()
                self.state = next_state
                self.is_aligned = False
                self._align_start_t = self.get_clock().now().nanoseconds / 1e9
                self.grab_confirm_count = 0
                self._smooth_x = None
                self._ball_history.clear()
                self._ball_still_count = 0
                return

        now = self.get_clock().now().nanoseconds / 1e9
        self._ball_history.append((now, msg.x, msg.z))

    # =====================================================
    # BOTTLE CALLBACK
    # =====================================================
    def bottle_callback(self, msg):
        if not self.is_active: return
        if math.isnan(msg.z) or msg.z <= 0.0: return

        if self.state == 'SEARCHING_BOTTLE':
            self._bottle_confirm_count += 1
            self.get_logger().info(
                f"🧴 Bottle seen ({self._bottle_confirm_count}/{self.BOTTLE_CONFIRM_NEEDED})")
            if self._bottle_confirm_count < self.BOTTLE_CONFIRM_NEEDED:
                self.stop_robot()
                self._search_spinning = False
                if self._search_timer is not None:
                    self._search_timer.cancel()
                    self._search_timer = None
                return
            self.get_logger().info("🧴 BOTTLE CONFIRMED!")
            self._bottle_confirm_count = 0
            self.state = 'CHASING_BOTTLE'
            return

        if self.state != 'CHASING_BOTTLE': return

        drop_dist = 0.6

        if msg.z <= drop_dist and abs(msg.x) < 0.15:
            self.execute_drop()
            return

        err_dist  = msg.z - drop_dist
        target_vx = max(0.0, min(0.4, 0.6 * err_dist))
        target_wz = max(-0.18, min(0.18, -0.25 * msg.x))

        self.current_vx = target_vx  # no ramp — accuracy not needed, start moving immediately
        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.angular.z = target_wz
        self.pub_cmd.publish(cmd)

    def _bottle_confirm_reset_cb(self):
        pass

    # =====================================================
    # SEARCH SPIN HELPERS
    # =====================================================
    def _search_pause_cb(self):
        if self._search_timer is not None:
            self._search_timer.cancel()
            self._search_timer = None
        self.stop_robot()
        self._search_timer = self.create_timer(0.3, self._search_resume_cb)

    def _search_resume_cb(self):
        if self._search_timer is not None:
            self._search_timer.cancel()
            self._search_timer = None
        self._search_spinning = False

    # =====================================================
    # GRAB / DROP SEQUENCES
    # =====================================================
    def execute_grab(self):
        self.get_logger().info("======================================")
        self.get_logger().info("🎾 BALL SECURED — Executing Grab...")
        self.state = 'GRABBING'
        self.stop_robot()
        self.current_vx = 0.0
        self.is_aligned = False
        self._align_start_t = self.get_clock().now().nanoseconds / 1e9
        self.publish_claw(self.CLAW_OPEN)
        self._target_arm_angle = self.ARM_MAX_TILT
        self._arm_step_dir     = -1.0
        self._active_action    = 'GRAB'
        self._arm_sweep_timer  = self.create_timer(0.02, self._sweep_arm_cb)

    def execute_drop(self):
        self.get_logger().info("======================================")
        self.get_logger().info("🧴 BOTTLE REACHED — Executing Drop...")
        self.state = 'DROPPING'
        self.stop_robot()
        self._target_arm_angle = self.ARM_MAX_TILT
        self._arm_step_dir     = -1.0
        self._active_action    = 'DROP'
        self._arm_sweep_timer  = self.create_timer(0.02, self._sweep_arm_cb)

    def _sweep_arm_cb(self):
        step_size = 1.0
        self.arm_angle += step_size * self._arm_step_dir

        if (self._arm_step_dir < 0 and self.arm_angle <= self._target_arm_angle) or \
           (self._arm_step_dir > 0 and self.arm_angle >= self._target_arm_angle):

            self.arm_angle = self._target_arm_angle
            self.publish_arm_angle(self.arm_angle)
            self._arm_sweep_timer.cancel()

            if self.arm_angle == self.ARM_MAX_TILT:
                if self._active_action == 'GRAB':
                    self._action_timer = self.create_timer(0.5, self._close_claw_cb)
                else:
                    self._action_timer = self.create_timer(0.5, self._open_claw_drop_cb)
            else:
                if self._active_action == 'GRAB':
                    self.get_logger().info("✅ Grab complete — turning to bottle...")
                    self._begin_turn_to_bottle()
                else:
                    self.get_logger().info("✅ Drop complete — returning to court...")
                    self._drop_position = (self.current_x, self.current_y)
                    self._begin_turn_to_court()
        else:
            self.publish_arm_angle(self.arm_angle)

    def _begin_turn_to_bottle(self):
        """After grab — compute best angle to bottle and begin turning."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', 'neon_bottle',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            bx = tf.transform.translation.x
            bz = tf.transform.translation.z
            angle = math.atan2(bx, bz)
            self._bottle_target_angle = self._wrap(self.current_yaw + angle)
            self.get_logger().info(f"🧴 Fresh bottle TF: turning {math.degrees(angle):.1f}°")
            self._turn_start_yaw = None
            self.state = 'TURNING_TO_BOTTLE'
        except Exception as e:
            self.get_logger().warn(f"⚠️  Bottle TF failed ({e}) — sweeping...")
            self._search_spinning = False
            self._turn_start_yaw  = None
            self.state = 'SEARCHING_BOTTLE'

    def _begin_turn_to_court(self):
        """After drop — return to face the court.

        USE_POLE_DETECTION = True  → rotate left until the court pole is seen,
                                     then nudge right POLE_ALIGN_DEG degrees.
        USE_POLE_DETECTION = False → rotate a fixed RETURN_TURN_DEG (for testing).
        """
        if self.USE_POLE_DETECTION:
            self._pole_confirm_count = 0
            self._pole_align_yaw    = None
            self._turn_settle_since = None
            self.state = 'SEARCHING_COURT_POLE'
            self.get_logger().info("🔍 Searching for court pole (rotating left)...")
        else:
            self.court_yaw = self._wrap(
                self.current_yaw + math.radians(self.RETURN_TURN_DEG))
            self._turn_settle_since = None
            self._turn_start_yaw    = None
            self.state = 'TURNING_TO_COURT'
            self.get_logger().info(
                f"↩️ Turning {self.RETURN_TURN_DEG}° from current → "
                f"court yaw={math.degrees(self.court_yaw):.1f}°...")

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
        self._arm_step_dir     = 1.0
        self._arm_sweep_timer  = self.create_timer(0.02, self._sweep_arm_cb)

    # =====================================================
    # UTILS
    # =====================================================
    def _wrap(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _adiff(self, target, current):
        return self._wrap(target - current)

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
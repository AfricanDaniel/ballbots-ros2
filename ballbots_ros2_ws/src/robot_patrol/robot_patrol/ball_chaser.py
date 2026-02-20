#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Float32
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
            ]
        )

        # =====================================================
        # LOAD PARAMETERS
        # =====================================================
        self.ARM_HORIZONTAL  = self.get_parameter('arm_raised_deg').value
        self.ARM_MAX_TILT    = self.get_parameter('arm_lowered_deg').value
        self.CLAW_OPEN       = self.get_parameter('claw_open_deg').value
        self.CLAW_CLOSED     = self.get_parameter('claw_closed_deg').value
        self.target_dist     = self.get_parameter('target_dist').value
        self.tilt_start_dist = self.get_parameter('tilt_start_dist').value

        self.get_logger().info(
            f"Params loaded | ARM_HORIZONTAL={self.ARM_HORIZONTAL} | ARM_MAX_TILT={self.ARM_MAX_TILT} | "
            f"CLAW_OPEN={self.CLAW_OPEN} | CLAW_CLOSED={self.CLAW_CLOSED} | "
            f"target_dist={self.target_dist} | tilt_start_dist={self.tilt_start_dist}"
        )

        # =====================================================
        # SUBSCRIBERS / PUBLISHERS
        # =====================================================
        self.sub_cam = self.create_subscription(
            Point, '/tennis_ball_position', self.listener_callback, 10)

        self.sub_cam_close = self.create_subscription(
            Point, '/tennis_ball_position_close', self.close_range_callback, 10)

        self.pub_cmd    = self.create_publisher(Twist,   '/cmd_vel',      10)
        self.pub_arm    = self.create_publisher(Float32, '/arm_command',  10)
        self.pub_claw   = self.create_publisher(Float32, '/claw_command', 10)
        self.pub_signal = self.create_publisher(Bool,    '/ball_ready',   10)

        # =====================================================
        # SERVICES
        # =====================================================
        self.srv_start = self.create_service(SetBool, 'start_chasing', self.handle_start)
        self.srv_stop  = self.create_service(SetBool, 'stop_chasing',  self.handle_stop)
        self.srv_estop = self.create_service(SetBool, 'estop',         self.handle_estop)

        # =====================================================
        # STATE
        # =====================================================
        self.is_active         = False
        self.arm_angle         = self.ARM_HORIZONTAL
        self.claw_opened       = False
        self.use_close_cam     = False
        self.ball_grabbed      = False
        self.grab_started      = False
        self.last_realsense_time = None
        self.grab_confirm_count  = 0
        self.GRAB_CONFIRM_NEEDED = 3
        self._creep_timer        = None
        self._last_zed_vx = 0.0
        self._last_zed_vy = 0.0

        self.is_aligned = False
        self.current_vx = 0.0
        self.MAX_ACCEL = 0.02

        # Watchdog disabled — using timed creep instead
        self.create_timer(1.0, self.realsense_watchdog_cb)

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
            self.grab_started        = False
            self.claw_opened         = False
            self.use_close_cam       = False
            self.ball_grabbed        = False
            self.last_realsense_time = None
            self.grab_confirm_count  = 0
            self._cancel_creep_timer()
            self.is_aligned = False
            msg = "Chasing Started!"
        else:
            self.publish_arm_angle(self.ARM_HORIZONTAL)
            self.stop_robot()
            self._cancel_creep_timer()
            msg = "Chasing Stopped."

        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    def handle_stop(self, request, response):
        return self.handle_start(request, response)

    def handle_estop(self, request, response):
        self.is_active           = False
        self.grab_started        = False
        self.use_close_cam       = False
        self.ball_grabbed        = False
        self.last_realsense_time = None
        self.grab_confirm_count  = 0
        self._cancel_creep_timer()
        self.stop_robot()
        response.success = True
        response.message = "E-Stop triggered"
        self.get_logger().info("🛑 E-Stop!")
        return response

    # =====================================================
    # HELPERS
    # =====================================================
    def _cancel_creep_timer(self):
        if self._creep_timer is not None:
            self._creep_timer.cancel()
            self._creep_timer = None

    def publish_arm_angle(self, angle):
        self.arm_angle = angle
        msg = Float32()
        msg.data = float(angle)
        self.pub_arm.publish(msg)

    def publish_claw(self, value):
        msg = Float32()
        msg.data = float(value)
        self.pub_claw.publish(msg)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

    # =====================================================
    # WATCHDOG — disabled, kept for reference
    # =====================================================
    def realsense_watchdog_cb(self):
        return  # disabled — using timed creep on switch instead

    # =====================================================
    # GRAB SEQUENCE
    # Step 1: Open claw
    # Step 2: Lower arm to ARM_MAX_TILT
    # Step 3: Close claw (after 1s delay)
    # Step 4: Raise arm back to horizontal
    # =====================================================
    # =====================================================
    # GRAB SEQUENCE (Smooth Interpolation)
    # =====================================================
    def execute_grab(self):
        if self.ball_grabbed:
            return

        self.get_logger().info("🎾 ========================================")
        self.get_logger().info("🎾 BALL GRABBED — executing smooth grab sequence")
        self.get_logger().info("🎾 ========================================")

        self.stop_robot()
        self.ball_grabbed = True
        self.grab_started = True

        # Step 1: Open claw
        self.publish_claw(self.CLAW_OPEN)

        # Step 2: Start smoothly lowering the arm
        self._target_arm_angle = self.ARM_MAX_TILT
        self._arm_step_dir = -1.0  # Direction multiplier (moving down)
        self._arm_sweep_timer = self.create_timer(0.02, self._sweep_arm_cb)  # 50Hz update rate

    def _sweep_arm_cb(self):
        """Slowly moves the arm 1 degree per tick until it hits the target"""
        step_size = 1.0  # Move 1 degree every 0.02s
        self.arm_angle += (step_size * self._arm_step_dir)

        # Check if we have reached or passed the target angle
        if (self._arm_step_dir < 0 and self.arm_angle <= self._target_arm_angle) or \
                (self._arm_step_dir > 0 and self.arm_angle >= self._target_arm_angle):

            self.arm_angle = self._target_arm_angle
            self.publish_arm_angle(self.arm_angle)
            self._arm_sweep_timer.cancel()

            # State routing: Did we just finish going down, or up?
            if self.arm_angle == self.ARM_MAX_TILT:
                # Reached the bottom. Wait 0.5s for stability, then close claw.
                self._grab_timer = self.create_timer(0.5, self._close_claw_cb)
            else:
                # Reached the top. Sequence is totally finished.
                self.pub_signal.publish(Bool(data=True))
                self.get_logger().info("✅ Smooth grab sequence complete")
        else:
            # Haven't reached target yet, keep publishing the intermediate angle
            self.publish_arm_angle(self.arm_angle)

    def _close_claw_cb(self):
        self._grab_timer.cancel()
        self.get_logger().info(f"Grab step 3: Closing claw to {self.CLAW_CLOSED}°")
        self.publish_claw(self.CLAW_CLOSED)

        # Wait 1.0s for the ball to be fully gripped, then start smooth raise
        self._raise_timer = self.create_timer(1.0, self._start_raise_arm_cb)

    def _start_raise_arm_cb(self):
        self._raise_timer.cancel()
        self.get_logger().info(f"Grab step 4: Smoothly raising arm to {self.ARM_HORIZONTAL}°")

        # Set target to top and start sweeping up
        self._target_arm_angle = self.ARM_HORIZONTAL
        self._arm_step_dir = 1.0  # Direction multiplier (moving up)
        self._arm_sweep_timer = self.create_timer(0.02, self._sweep_arm_cb)

    # =====================================================
    # CREEP DONE
    # =====================================================
    def _creep_done_cb(self):
        self._cancel_creep_timer()
        self.get_logger().info("Creep done — stopping and executing grab")
        self.stop_robot()
        self.execute_grab()

    # =====================================================
    # ZED CALLBACK (far range: > tilt_start_dist)
    # =====================================================
    def listener_callback(self, msg):
        if not self.is_active:
            return

        if self.ball_grabbed:
            return

        if math.isnan(msg.z) or msg.z <= 0.0:
            self.stop_robot()
            return

        # Once use_close_cam is set, NEVER reset it from the ZED callback
        # (prevents flag flipping if ZED sends a stale far reading after switch)
        if self.use_close_cam:
            return

        # Hand off to RealSense when close — creep forward then grab
        if msg.z < self.tilt_start_dist:
            self.get_logger().info(
                f"ZED z={msg.z:.2f}m < tilt_start_dist={self.tilt_start_dist:.2f}m "
                f"— switching to RealSense, creeping forward"
            )
            self.use_close_cam = True

            # Open claw before creeping
            self.publish_claw(self.CLAW_OPEN)

            # Creep in same direction as last ZED command
            cmd = Twist()
            cmd.linear.x = self._last_zed_vx * 0.3
            cmd.linear.y = self._last_zed_vy * 0.3
            self.get_logger().info(
                f"Creeping at vx={cmd.linear.x:.2f} vy={cmd.linear.y:.2f} for ??s"
            )
            self.pub_cmd.publish(cmd)
            self._creep_timer = self.create_timer(7.0, self._creep_done_cb)
            return

        # Far range — arm horizontal, approach ball
        self.publish_arm_angle(self.ARM_HORIZONTAL)

        # =====================================================
        # HSV STATE MACHINE: ALIGN THEN DRIVE
        # =====================================================
        target_vx = 0.0
        target_wz = 0.0
        err_dist = msg.z - self.target_dist

        # 🚨 TRIPWIRE REMOVED: The robot will no longer stop to re-align!

        # STATE 1: ALIGNING PHASE
        if not self.is_aligned:
            target_vx = 0.0  # Brakes applied

            # Faster multiplier because HSV has no latency
            target_wz = max(-0.4, min(0.4, -1.0 * msg.x))

            # Tight 5cm window (HSV can easily hit this)
            if abs(msg.x) < 0.05:
                self.get_logger().info("Aligned! Driving forward with micro-corrections.")
                self.is_aligned = True

        # STATE 2: DRIVING PHASE
        else:
            # Drive straight towards the distance detected
            target_vx = max(-0.3, min(0.3, 0.5 * err_dist))

            # MICRO-CORRECTIONS: Steer gently while driving to fight mecanum drift
            target_wz = max(-0.15, min(0.15, -0.5 * msg.x))

        # APPLY SMOOTHER (Only to linear forward/back to prevent wheel slip)
        self.current_vx += max(-self.MAX_ACCEL, min(self.MAX_ACCEL, target_vx - self.current_vx))

        cmd = Twist()
        cmd.linear.x = self.current_vx
        cmd.linear.y = 0.0  # Strictly zero to prevent crab-walking
        cmd.angular.z = target_wz

        # Save for the RealSense creep handoff
        self._last_zed_vx = cmd.linear.x
        self._last_zed_vy = 0.0

        self.pub_cmd.publish(cmd)

    # =====================================================
    # REALSENSE CALLBACK (close range: < tilt_start_dist)
    # Kept for logging/future use — motion is now timed creep
    # =====================================================
    def close_range_callback(self, msg):
        self.get_logger().info("Entered real sense territory")
        return
        if not self.is_active:
            return

        if not self.use_close_cam:
            return

        if self.ball_grabbed:
            return

        # Update watchdog time only on valid readings
        if math.isnan(msg.z) or msg.z <= 0.0:
            return

        if msg.z > self.tilt_start_dist:
            self.get_logger().warn(
                f"RealSense z={msg.z:.2f}m > tilt_start_dist — ignoring bad reading",
                throttle_duration_sec=1.0
            )
            return

        # Valid reading — update time
        self.last_realsense_time = self.get_clock().now()

        # =====================================================
        # BLIND SPOT GRAB — add this RIGHT HERE, before anything else
        # Ball is in RealSense blind spot (<15cm) and centered
        # Trigger grab immediately instead of waiting for real_dist
        # =====================================================
        if msg.z == 0.15 and abs(msg.x) < 0.06:
            self.grab_confirm_count += 1
            self.get_logger().info(
                f"Blind spot grab confirm {self.grab_confirm_count}/{self.GRAB_CONFIRM_NEEDED} "
                f"(x={msg.x:.3f}m)"
            )
            if self.grab_confirm_count >= self.GRAB_CONFIRM_NEEDED:
                self._cancel_creep_timer()
                self.execute_grab()
            return
        # =====================================================

        # Real Distance Compensation
        pitch_deg = self.ARM_HORIZONTAL - self.arm_angle
        pitch_rad = math.radians(pitch_deg)
        real_dist = msg.z * math.cos(pitch_rad)

        self.get_logger().info(
            f"RealSense: z={msg.z:.2f}m real_dist={real_dist:.2f}m x={msg.x:.2f}m",
            throttle_duration_sec=0.5
        )

        # Tilt arm progressively
        ratio = (real_dist - self.target_dist) / (self.tilt_start_dist - self.target_dist)
        ratio = max(0.0, min(1.0, ratio))
        target_angle = self.ARM_MAX_TILT + (ratio * (self.ARM_HORIZONTAL - self.ARM_MAX_TILT))

        if real_dist < 0.30:
            self.publish_arm_angle(target_angle)
        else:
            new_angle = self.arm_angle + 0.2 * (target_angle - self.arm_angle)
            self.publish_arm_angle(new_angle)

        # Motion disabled — timed creep handles movement
        # cmd = Twist()
        # err_dist = real_dist - self.target_dist
        # if abs(err_dist) > 0.05:
        #     cmd.linear.x = max(-0.3, min(0.3, 0.5 * err_dist))
        #     cmd.linear.y = max(-0.3, min(0.3, -1.0 * msg.x))
        # self.pub_cmd.publish(cmd)


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
        node.get_logger().info("Shutting down — stopping robot...")
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
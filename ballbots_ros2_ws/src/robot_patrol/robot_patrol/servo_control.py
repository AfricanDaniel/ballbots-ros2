#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from minimec_msgs.srv import ServoCommand
from std_msgs.msg import Bool, Float32
import sys
import time
import threading

# -------------------------------
# Dynamixel Protocol Constants
# -------------------------------
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


# -------------------------------
# Conversion Helpers
# -------------------------------
def deg2unit(deg):
    return int(deg / 360.0 * 4095)


def unit2deg(unit):
    return unit / 4095.0 * 360


# ===============================
# Servo Control Node
# ===============================
class ServoControlNode(Node):

    def __init__(self):
        super().__init__('servo_control_node')

        # =====================================================
        # DECLARE PARAMETERS (Loaded from YAML)
        # =====================================================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device_name', '/dev/ttyUSB0'),
                ('baudrate', 57600),
                ('arm_id', 0),
                ('claw_id', 1),

                ('arm_raised_deg', 252.0),
                ('arm_lowered_deg', 177.0),

                ('claw_open_deg', 153.0),
                ('claw_closed_deg', 173.0),

                ('target_dist', 0.35),
                ('tilt_start_dist', 0.8),
            ]
        )

        # =====================================================
        # LOAD PARAMETERS
        # =====================================================
        self.device_name = self.get_parameter('device_name').value
        self.baudrate = self.get_parameter('baudrate').value
        self.ARM_ID = self.get_parameter('arm_id').value
        self.CLAW_ID = self.get_parameter('claw_id').value

        self.ARM_RAISED = self.get_parameter('arm_raised_deg').value
        self.ARM_LOWERED = self.get_parameter('arm_lowered_deg').value

        self.CLAW_OPEN = self.get_parameter('claw_open_deg').value
        self.CLAW_CLOSED = self.get_parameter('claw_closed_deg').value

        self.TARGET_DIST = self.get_parameter('target_dist').value
        self.TILT_START_DIST = self.get_parameter('tilt_start_dist').value

        # =====================================================
        # Dynamixel Setup
        # =====================================================
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port {self.device_name}")
            sys.exit(1)

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error(f"Failed to set baudrate {self.baudrate}")
            sys.exit(1)

        self.get_logger().info("Dynamixel Port Opened.")

        # Enable torque
        for servo_id in [self.ARM_ID, self.CLAW_ID]:
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE
            )

        # =====================================================
        # STATE
        # =====================================================
        self.arm_state = "STARTUP"
        self.lock = threading.Lock()

        # =====================================================
        # SUBSCRIBERS
        # =====================================================
        self.angle_sub = self.create_subscription(
            Float32,
            '/arm_command',
            self.arm_command_callback,
            10
        )

        self.ball_signal_sub = self.create_subscription(
            Bool,
            '/ball_ready',
            self.ball_signal_callback,
            10
        )

        # =====================================================
        # SERVICES
        # =====================================================
        self.srv_move = self.create_service(
            ServoCommand,
            'move_servo',
            self.handle_manual_move
        )

        self.srv_reboot = self.create_service(
            ServoCommand,
            'reboot_servo',
            self.handle_reboot
        )

        # =====================================================
        # STARTUP SEQUENCE
        # =====================================================
        threading.Thread(target=self.startup_sequence, daemon=True).start()

    # =====================================================
    # STARTUP
    # =====================================================
    def startup_sequence(self):
        self.get_logger().info("Startup: Closing claw and raising arm...")

        self.move_slowly(self.CLAW_ID, self.CLAW_CLOSED)
        self.move_slowly(self.ARM_ID, self.ARM_RAISED)

        with self.lock:
            self.arm_state = "READY"

        self.get_logger().info("Startup Complete.")

    # =====================================================
    # TRACKING CALLBACK
    # =====================================================
    def arm_command_callback(self, msg):

        with self.lock:
            if self.arm_state == "BUSY":
                return
            self.arm_state = "TRACKING"

        target = msg.data

        # Safety clamp using config limits
        target = max(self.ARM_LOWERED, min(self.ARM_RAISED, target))

        self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self.ARM_ID,
            ADDR_GOAL_POSITION,
            deg2unit(target)
        )

    # =====================================================
    # BALL GRAB
    # =====================================================
    def ball_signal_callback(self, msg):
        if not msg.data:
            return

        with self.lock:
            if self.arm_state == "BUSY":
                return
            self.arm_state = "BUSY"

        self.get_logger().info("Ball ready signal received. Grabbing...")
        threading.Thread(target=self.perform_grab, daemon=True).start()

    def perform_grab(self):

        self.move_slowly(self.CLAW_ID, self.CLAW_OPEN)
        time.sleep(0.5)
        self.move_slowly(self.CLAW_ID, self.CLAW_CLOSED)

        with self.lock:
            self.arm_state = "READY"

        self.get_logger().info("Grab complete.")

    # =====================================================
    # SMOOTH MOVE
    # =====================================================
    def move_slowly(self, servo_id, target_deg, speed_deg_per_sec=30.0):

        pos_data, res, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler,
            servo_id,
            ADDR_PRESENT_POSITION
        )

        if res != COMM_SUCCESS:
            return False

        current_deg = unit2deg(pos_data)
        diff = target_deg - current_deg

        if abs(diff) < 1.0:
            return True

        steps = int((abs(diff) / speed_deg_per_sec) * 20)
        if steps < 1:
            steps = 1

        step_deg = diff / steps

        for i in range(steps):
            goal = current_deg + (step_deg * (i + 1))
            self.packet_handler.write4ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_GOAL_POSITION,
                deg2unit(goal)
            )
            time.sleep(0.05)

        self.packet_handler.write4ByteTxRx(
            self.port_handler,
            servo_id,
            ADDR_GOAL_POSITION,
            deg2unit(target_deg)
        )

        return True

    # =====================================================
    # MANUAL SERVICE
    # =====================================================
    def handle_manual_move(self, request, response):

        pos_data, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler,
            request.servo_id,
            ADDR_PRESENT_POSITION
        )

        current = unit2deg(pos_data)
        target = current + request.delta_degrees
        target = max(0.0, min(360.0, target))

        success = self.move_slowly(request.servo_id, target)

        response.success = success
        return response

    # =====================================================
    # REBOOT SERVICE
    # =====================================================
    def handle_reboot(self, request, response):

        self.packet_handler.reboot(self.port_handler, request.servo_id)
        time.sleep(1.0)

        self.packet_handler.write1ByteTxRx(
            self.port_handler,
            request.servo_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ENABLE
        )

        response.success = True
        return response

    # =====================================================
    # SHUTDOWN
    # =====================================================
    def shutdown(self):

        self.get_logger().info("Shutdown: Lowering arm before disabling torque...")

        # Prevent tracking during shutdown
        with self.lock:
            self.arm_state = "BUSY"

        # 1️⃣ Lower arm safely
        self.move_slowly(self.ARM_ID, self.ARM_LOWERED)

        time.sleep(0.5)  # Small safety delay

        # 2️⃣ Disable torque
        for servo_id in [self.ARM_ID, self.CLAW_ID]:
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE
            )

        # 3️⃣ Close port
        self.port_handler.closePort()

        self.get_logger().info("Shutdown complete.")


# =====================================================
# MAIN
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
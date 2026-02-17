import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from minimec_msgs.srv import ServoCommand
from std_msgs.msg import Bool
import sys
import time
import threading

# -------------------------------
# CONFIGURATION
# -------------------------------
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11  # Kept for completeness
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# --- USER POSITIONS ---
ARM_ID = 0
CLAW_ID = 1

# Arm Positions
ARM_RAISED = 252.0
ARM_LOWERED = 177.0

# Claw Positions
CLAW_OPEN = 153.0
CLAW_CLOSED = 173.0


def deg2unit(deg):
    return int(deg / 360.0 * 4095)


def unit2deg(unit):
    return unit / 4095.0 * 360


class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')

        # 1. Setup Dynamixel Connection
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port {DEVICENAME}")
            sys.exit(1)

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error(f"Failed to set baudrate {BAUDRATE}")
            sys.exit(1)

        self.get_logger().info("Dynamixel Port Opened.")

        # Enable Torque
        for servo_id in [ARM_ID, CLAW_ID]:
            self.packet_handler.write1ByteTxRx(self.port_handler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        # 2. State Tracking
        self.arm_state = "UNKNOWN"  # States: UNKNOWN, READY, GRABBING, BUSY

        # 3. Subscribers & Services
        # Listen for the signal from ball_chaser
        self.subscription = self.create_subscription(
            Bool,
            '/ball_ready',
            self.ball_signal_callback,
            10
        )

        # Legacy Service 1: Manual Move (Delta)
        self.srv_move = self.create_service(
            ServoCommand,
            'move_servo',
            self.handle_manual_move
        )
        self.get_logger().info("Ready! Service: /move_servo")

        # Legacy Service 2: Reboot Servo
        self.srv_reboot = self.create_service(
            ServoCommand,
            'reboot_servo',
            self.handle_reboot
        )
        self.get_logger().info("Ready! Service: /reboot_servo")

        # 4. Startup Sequence (Run in separate thread to not block ROS)
        self.get_logger().info("Startup: Moving to RAISED and CLOSED position...")
        # daemon=True means "Kill this thread if the program quits"
        threading.Thread(target=self.startup_sequence, daemon=True).start()


    def startup_sequence(self):
        """Moves arm up and claw closed slowly on startup"""
        # Close Claw first
        self.move_slowly(CLAW_ID, CLAW_CLOSED, speed_deg_per_sec=30.0)
        # Raise Arm
        self.move_slowly(ARM_ID, ARM_RAISED, speed_deg_per_sec=30.0)

        self.arm_state = "READY"
        self.get_logger().info("Startup Complete. Waiting for /ball_ready signal.")

    def ball_signal_callback(self, msg):
        """Triggers when ball_chaser says 'True'"""
        if msg.data is True and self.arm_state == "READY":
            self.get_logger().info("Signal Received! Initiating Grab Sequence...")
            self.arm_state = "BUSY"

            # Start the sequence in a thread
            threading.Thread(target=self.perform_grab, daemon=True).start()

    def perform_grab(self):
        """The sequence: Move Down -> Open Claw"""
        # 1. Move Arm Down Slowly
        self.get_logger().info("Lowering Arm...")
        success = self.move_slowly(ARM_ID, ARM_LOWERED, speed_deg_per_sec=20.0)

        if success:
            # 2. Open Claw Slowly
            self.get_logger().info("Opening Claw...")
            self.move_slowly(CLAW_ID, CLAW_OPEN, speed_deg_per_sec=30.0)
            self.arm_state = "GRABBING"
            self.get_logger().info("Sequence Finished: Arm Lowered & Claw Open.")

    def move_slowly(self, servo_id, target_deg, speed_deg_per_sec=30.0):
        """Interpolates movement to avoid jerking"""
        # Read current position
        pos_data, res, err = self.packet_handler.read4ByteTxRx(
            self.port_handler, servo_id, ADDR_PRESENT_POSITION
        )
        if res != COMM_SUCCESS:
            self.get_logger().error(f"Read Error on ID {servo_id}")
            return False

        current_deg = unit2deg(pos_data)

        # Calculate steps
        diff = target_deg - current_deg
        if abs(diff) < 1.0: return True  # Already there

        duration = abs(diff) / speed_deg_per_sec
        steps = int(duration * 20)  # 20 Hz update rate
        if steps < 1: steps = 1

        step_deg = diff / steps

        for i in range(steps):
            goal = current_deg + (step_deg * (i + 1))
            goal_unit = deg2unit(goal)

            self.packet_handler.write4ByteTxRx(
                self.port_handler, servo_id, ADDR_GOAL_POSITION, goal_unit
            )
            time.sleep(0.05)  # 20Hz (50ms delay)

        # Final ensure
        self.packet_handler.write4ByteTxRx(
            self.port_handler, servo_id, ADDR_GOAL_POSITION, deg2unit(target_deg)
        )
        return True

    def handle_manual_move(self, request, response):
        """Legacy service to move by delta"""
        servo_id = request.servo_id
        delta = request.delta_degrees

        # Read current position first to calculate the target
        pos_data, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, servo_id, ADDR_PRESENT_POSITION)
        current = unit2deg(pos_data)
        target = current + delta
        target = max(0.0, min(360.0, target))  # Clamp

        # Move it slowly to avoid jerking even on manual calls
        success = self.move_slowly(servo_id, target, speed_deg_per_sec=40.0)

        if success:
            response.success = True
            response.message = f"Servo {servo_id} moved {delta}°. Now at {target:.2f}°"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Failed to move servo {servo_id}"
        return response

    def handle_reboot(self, request, response):
        """Legacy service to reboot a servo and clear hardware errors"""
        servo_id = request.servo_id
        self.get_logger().info(f"Attempting to REBOOT servo {servo_id}...")

        # 1. Send Reboot Command
        res, err = self.packet_handler.reboot(self.port_handler, servo_id)
        if res != COMM_SUCCESS:
            response.success = False
            response.message = f"Reboot Failed: {self.packet_handler.getTxRxResult(res)}"
            return response

        # 2. Wait for it to wake up
        time.sleep(1.0)

        # 3. Re-Enable Torque
        res, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if res != COMM_SUCCESS:
            response.success = False
            response.message = f"Rebooted, but torque failed: {self.packet_handler.getTxRxResult(res)}"
        else:
            response.success = True
            response.message = f"Servo {servo_id} successfully rebooted and re-enabled!"
            self.get_logger().info(response.message)

        return response

    def shutdown(self):
        self.get_logger().info("Shutting down: Disabling torque on servos...")

        for servo_id in [ARM_ID, CLAW_ID]:
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                servo_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE
            )

        time.sleep(0.2)  # small delay to ensure packets are sent
        self.port_handler.closePort()
        self.get_logger().info("Dynamixel torque disabled and port closed.")


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
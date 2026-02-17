#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from std_srvs.srv import SetBool
import sys
import time

# -------------------------------
# CONFIGURATION
# -------------------------------
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
SERVO_IDS = [0, 1]
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


# Degrees ↔ Units conversion
def deg2unit(deg):
    return int(deg / 360.0 * 4095)


def unit2deg(unit):
    return unit / 4095.0 * 360


# -------------------------------
# NODE
# -------------------------------
class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')

        # Initialize port & packet handlers
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port {DEVICENAME}")
            sys.exit(1)

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error(f"Failed to set baudrate {BAUDRATE}")
            sys.exit(1)

        self.get_logger().info("Port opened and baudrate set successfully")

        # Check operating mode
        for dxl_id in SERVO_IDS:
            mode, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE
            )
            self.get_logger().info(f"Servo {dxl_id} mode: {mode} (3=Position Control)")

        # Enable torque on both servos
        for dxl_id in SERVO_IDS:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
        self.get_logger().info("Torque enabled on all servos")

        # Read current positions - DO NOT MOVE SERVOS
        for dxl_id in SERVO_IDS:
            present_position, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, dxl_id, ADDR_PRESENT_POSITION
            )
            current_deg = unit2deg(present_position)
            self.get_logger().info(
                f"Servo {dxl_id} startup position: {current_deg:.2f}° "
                f"(no movement - position saved as reference)"
            )

        # Create services for each servo
        self.srv_servo0 = self.create_service(
            SetBool, 'move_servo_0', self.handle_servo_0
        )
        self.srv_servo1 = self.create_service(
            SetBool, 'move_servo_1', self.handle_servo_1
        )

        # Store delta degrees (you can modify this via parameter)
        self.declare_parameter('delta_degrees', 10.0)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Servo control node ready!")
        self.get_logger().info("Servos WILL NOT MOVE on startup")
        self.get_logger().info("Range: 0-360° with hard stops at limits")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Usage:")
        self.get_logger().info("  ros2 service call /move_servo_0 std_srvs/srv/SetBool \"{data: true}\"  # +10°")
        self.get_logger().info("  ros2 service call /move_servo_0 std_srvs/srv/SetBool \"{data: false}\" # -10°")
        self.get_logger().info("  ros2 param set /servo_control_node delta_degrees 5.0  # change increment")

    def move_servo(self, servo_id, delta_deg):
        """Move servo by delta_deg degrees (clamped to 0-360°)"""
        try:
            # Read current position
            present_position, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, servo_id, ADDR_PRESENT_POSITION
            )
            current_deg = unit2deg(present_position)

            # Calculate new position with hard limits at 0° and 360°
            new_deg = current_deg + delta_deg
            new_deg = max(0.0, min(360.0, new_deg))  # Clamp to valid range
            goal_position = deg2unit(new_deg)

            # Send command
            self.packet_handler.write4ByteTxRx(
                self.port_handler, servo_id, ADDR_GOAL_POSITION, goal_position
            )

            self.get_logger().info(
                f"Servo {servo_id}: {current_deg:.2f}° → {new_deg:.2f}° "
                f"(Δ{delta_deg:+.2f}°)"
            )

            # Warn if hitting limits
            if new_deg == 0.0:
                self.get_logger().warn(f"Servo {servo_id} at LOWER LIMIT (0°)")
            elif new_deg == 360.0:
                self.get_logger().warn(f"Servo {servo_id} at UPPER LIMIT (360°)")

            return True, f"Moved servo {servo_id} to {new_deg:.2f}°"

        except Exception as e:
            self.get_logger().error(f"Error moving servo {servo_id}: {e}")
            return False, str(e)

    def handle_servo_0(self, request, response):
        delta = self.get_parameter('delta_degrees').value
        if not request.data:  # False = decrease
            delta = -delta
        response.success, response.message = self.move_servo(0, delta)
        return response

    def handle_servo_1(self, request, response):
        delta = self.get_parameter('delta_degrees').value
        if not request.data:  # False = decrease
            delta = -delta
        response.success, response.message = self.move_servo(1, delta)
        return response

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down, disabling torque on all servos...")
        for dxl_id in SERVO_IDS:
            try:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
                )
            except:
                pass
        self.port_handler.closePort()
        self.get_logger().info("Servos disabled. Goodbye!")


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
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class NumberPrinter(Node):
    def __init__(self):
        super().__init__('number_printer')
        self.counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s => 10Hz
        self.get_logger().info("NumberPrinter node started!")

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f"Number: {self.counter}")

def main(args=None):
    rclpy.init(args=args)
    node = NumberPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



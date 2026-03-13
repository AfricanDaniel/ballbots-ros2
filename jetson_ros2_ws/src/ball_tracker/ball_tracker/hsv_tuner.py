#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')

        # Default to ZED camera, but allow overriding via parameters
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/color/rect/image')
        topic_name = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10)

        cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("H_min", "Trackbars", 0, 179, lambda x: None)
        cv2.createTrackbar("H_max", "Trackbars", 179, 179, lambda x: None)
        cv2.createTrackbar("S_min", "Trackbars", 0, 255, lambda x: None)
        cv2.createTrackbar("S_max", "Trackbars", 255, 255, lambda x: None)
        cv2.createTrackbar("V_min", "Trackbars", 0, 255, lambda x: None)
        cv2.createTrackbar("V_max", "Trackbars", 255, 255, lambda x: None)

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"HSV Tuner listening to: {topic_name}")
        self.get_logger().info("Press 'p' on your keyboard to print current values.")
        self.get_logger().info("=" * 50)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h_min = cv2.getTrackbarPos("H_min", "Trackbars")
        h_max = cv2.getTrackbarPos("H_max", "Trackbars")
        s_min = cv2.getTrackbarPos("S_min", "Trackbars")
        s_max = cv2.getTrackbarPos("S_max", "Trackbars")
        v_min = cv2.getTrackbarPos("V_min", "Trackbars")
        v_max = cv2.getTrackbarPos("V_max", "Trackbars")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("Original", frame)
        cv2.imshow("Mask (White = Detected)", mask)
        cv2.imshow("Result", result)

        # Press 'p' to print the values to the terminal
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p'):
            print("\n--- COPY PASTE THESE ---")
            print(f"self.HSV_LOWER = ({h_min}, {s_min}, {v_min})")
            print(f"self.HSV_UPPER = ({h_max}, {s_max}, {v_max})")
            print("------------------------\n")


def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
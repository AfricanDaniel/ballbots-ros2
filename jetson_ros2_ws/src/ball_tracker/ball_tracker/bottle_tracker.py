#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from collections import deque


class BottleTracker(Node):
    def __init__(self):
        super().__init__('bottle_tracker')

        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, '/zed/zed_node/rgb/color/rect/image', self.rgb_cb, 10)
        self.depth_sub = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_cb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info',
                                                     self.cam_info_cb, 10)

        # Publishers
        self.bottle_pub = self.create_publisher(Point, '/bottle_position', 10)
        self.marker_pub = self.create_publisher(Marker, '/bottle_marker', 10)
        self.debug_img_pub = self.create_publisher(Image, '/bottle_debug_image', 10)

        self.fx = self.fy = self.cx = self.cy = None
        self.depth_img = None
        self.cam_info_received = False
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.frame_count = 0

        self.get_logger().info("=" * 60)
        self.get_logger().info("Pink Bottle Tracker node started! (With Wrap-Around HSV)")
        self.get_logger().info("=" * 60)

    def cam_info_cb(self, msg: CameraInfo):
        if self.cam_info_received: return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.cam_info_received = True

    def depth_cb(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_cb(self, msg: Image):
        if not self.cam_info_received:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        debug_img = cv_img.copy()
        h, w = cv_img.shape[:2]

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # ============================================================
        # COLOR PROFILER TOOL
        # Prints the HSV values of the exact center pixel of the screen
        # ============================================================
        center_hsv = hsv[h // 2, w // 2]
        self.frame_count += 1
        if self.frame_count % 30 == 0:  # Print once per second (assuming 30fps)
            self.get_logger().info(f"Center Crosshair HSV: H={center_hsv[0]}, S={center_hsv[1]}, V={center_hsv[2]}")

        # Draw a white crosshair in the center
        cv2.circle(debug_img, (w // 2, h // 2), 4, (255, 255, 255), -1)
        cv2.circle(debug_img, (w // 2, h // 2), 2, (0, 0, 0), -1)

        # ============================================================
        # NEON PINK / RED WRAP-AROUND HSV RANGE
        # ============================================================
        # Mask 1: Covers Hues 0 to 15
        mask1 = cv2.inRange(hsv, (0, 120, 100), (15, 255, 255))

        # Mask 2: Covers Hues 165 to 179
        mask2 = cv2.inRange(hsv, (165, 120, 100), (179, 255, 255))

        # Combine them together
        mask = cv2.bitwise_or(mask1, mask2)
        # ============================================================

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bottle_found = False

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area >= 50:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    u = int(M['m10'] / M['m00'])
                    v = int(M['m01'] / M['m00'])

                    # Draw what the color filter sees
                    cv2.drawContours(debug_img, [c], -1, (255, 0, 255), 2)
                    x_b, y_b, w_b, h_b = cv2.boundingRect(c)
                    cv2.rectangle(debug_img, (x_b, y_b), (x_b + w_b, y_b + h_b), (255, 0, 255), 2)
                    cv2.circle(debug_img, (u, v), 5, (0, 255, 0), -1)

                    # Try to get depth
                    if self.depth_img is not None:
                        try:
                            dh, dw = self.depth_img.shape[:2]
                            u_min, u_max = max(0, u - 2), min(dw, u + 3)
                            v_min, v_max = max(0, v - 2), min(dh, v + 3)
                            roi = self.depth_img[v_min:v_max, u_min:u_max]
                            valid = roi[np.isfinite(roi) & (roi > 0)]

                            if len(valid) > 0:
                                z = float(np.median(valid))
                                x_cam = (u - self.cx) * z / self.fx
                                y_cam = (v - self.cy) * z / self.fy

                                bottle_found = True

                                # ==========================================
                                # ADD THIS BLOCK TO PUBLISH THE TF FRAME
                                # ==========================================
                                t = TransformStamped()
                                t.header.stamp = self.get_clock().now().to_msg()
                                t.header.frame_id = 'zed_camera_center'
                                t.child_frame_id = 'neon_bottle'
                                t.transform.translation.x = float(x_cam)
                                t.transform.translation.y = float(y_cam)
                                t.transform.translation.z = float(z)
                                t.transform.rotation.x = 0.0
                                t.transform.rotation.y = 0.0
                                t.transform.rotation.z = 0.0
                                t.transform.rotation.w = 1.0
                                self.tf_broadcaster.sendTransform(t)
                                # ==========================================

                                # Publish Position
                                pt = Point()
                                pt.x, pt.y, pt.z = x_cam, y_cam, z
                                self.bottle_pub.publish(pt)

                                cv2.putText(debug_img, f'Pink Bottle ({z:.2f}m)', (x_b, y_b - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                                # Publish Marker for RViz
                                marker = Marker()
                                marker.header.frame_id = 'zed_camera_center'
                                marker.header.stamp = self.get_clock().now().to_msg()
                                marker.ns = 'pink_bottle'
                                marker.id = 1
                                marker.type = Marker.CYLINDER
                                marker.action = Marker.ADD
                                marker.pose.position.x = x_cam
                                marker.pose.position.y = y_cam
                                marker.pose.position.z = z
                                marker.pose.orientation.w = 1.0
                                marker.scale.x = 0.08
                                marker.scale.y = 0.08
                                marker.scale.z = 0.20
                                marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
                                marker.lifetime.sec = 1
                                self.marker_pub.publish(marker)

                            else:
                                cv2.putText(debug_img, 'No Depth', (x_b, y_b - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        except Exception:
                            pass

        if not bottle_found:
            cv2.putText(debug_img, 'Searching for Pink...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)

        # ALWAYS PUBLISH THE DEBUG IMAGE
        annotated_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        annotated_msg.header.stamp = self.get_clock().now().to_msg()
        annotated_msg.header.frame_id = 'zed_camera_center'
        self.debug_img_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BottleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
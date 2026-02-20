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

        # ============================================================
        # SHAPE VALIDATION - DISABLED FOR BOTTLE
        # ============================================================
        # A bottle is tall and cylindrical, not perfectly round.
        # We set this to False so it accepts the largest neon pink blob.
        self.ENABLE_VALIDATION = False
        # ============================================================

        # parameters to pull from launch file
        self.declare_parameter('debug_logs', False)
        self.debug_logs = self.get_parameter('debug_logs').value

        # CV bridge
        self.bridge = CvBridge()

        # Subscribers to ZED RGB and depth
        self.rgb_sub = self.create_subscription(
            Image, '/zed/zed_node/rgb/color/rect/image', self.rgb_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_cb, 10)

        # Subscriber for camera intrinsics
        self.cam_info_received = False
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info', self.cam_info_cb, 10)

        # Publishers for bottle location and visualization
        self.bottle_pub = self.create_publisher(Point, '/bottle_position', 10)
        self.marker_pub = self.create_publisher(Marker, '/bottle_marker', 10)
        self.annotated_img_pub = self.create_publisher(Image, '/bottle_detection_image', 10)
        self.debug_img_pub = self.create_publisher(Image, '/bottle_debug_image', 10)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.depth_img = None
        self.bottle_pos_px = None  # (u, v, area)

        # TF broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Motion tracking
        self.position_history = deque(maxlen=10)
        self.last_position = None
        self.last_timestamp = None
        self.MAX_VELOCITY_MPS = 10.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("Pink Bottle Tracker node started")
        self.get_logger().info("=" * 60)

    def cam_info_cb(self, msg: CameraInfo):
        if self.cam_info_received:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.cam_info_received = True

    def check_motion_pattern(self, current_pos):
        debug_info = {}
        current_time = self.get_clock().now()

        if self.last_position is not None and self.last_timestamp is not None:
            dt = (current_time - self.last_timestamp).nanoseconds / 1e9
            if dt > 0:
                displacement = np.linalg.norm(np.array(current_pos) - np.array(self.last_position))
                velocity = displacement / dt
                debug_info['velocity'] = velocity

                if velocity > self.MAX_VELOCITY_MPS:
                    debug_info['motion_warning'] = f'very_fast_movement ({velocity:.2f} m/s)'

        self.position_history.append(current_pos)
        self.last_position = current_pos
        self.last_timestamp = current_time
        return True, debug_info

    def publish_visual_marker(self, x_cam, y_cam, z_cam, u, v, contour, cv_img, is_valid=True):
        if is_valid:
            marker = Marker()
            marker.header.frame_id = 'zed_camera_center'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'pink_bottle'
            marker.id = 1
            marker.type = Marker.CYLINDER  # Changed from SPHERE to CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x_cam
            marker.pose.position.y = y_cam
            marker.pose.position.z = z_cam
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.08  # Approx 8cm wide
            marker.scale.y = 0.08  # Approx 8cm deep
            marker.scale.z = 0.20  # Approx 20cm tall
            marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)  # Neon Pink!
            marker.lifetime.sec = 1
            self.marker_pub.publish(marker)

        annotated_img = cv_img.copy()

        if is_valid:
            contour_color = (255, 0, 255)  # Pink bounding box
            bbox_color = (255, 0, 255)
            center_color = (0, 255, 0)
            label = f'Pink Bottle ({z_cam:.1f}m)'
        else:
            contour_color = (0, 0, 255)
            bbox_color = (0, 165, 255)
            center_color = (255, 0, 0)
            label = f'REJECTED ({z_cam:.1f}m)'

        cv2.drawContours(annotated_img, [contour], -1, contour_color, 2)
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(annotated_img, (x, y), (x + w, y + h), bbox_color, 2)
        cv2.circle(annotated_img, (u, v), 5, center_color, -1)
        cv2.putText(annotated_img, label, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, contour_color, 2)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
        annotated_msg.header.stamp = self.get_clock().now().to_msg()
        annotated_msg.header.frame_id = 'zed_camera_center'

        if is_valid:
            self.annotated_img_pub.publish(annotated_msg)
        self.debug_img_pub.publish(annotated_msg)

    def rgb_cb(self, msg: Image):
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # ============================================================
        # NEON PINK HSV RANGE
        # (OpenCV Hue goes 0-179. Pink/Magenta is around 140-170)
        # ============================================================
        mask = cv2.inRange(hsv, (135, 50, 50), (175, 255, 255))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < 50:  # Slightly larger minimum area for a bottle
                return

            M = cv2.moments(c)
            if M['m00'] > 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])

                self.bottle_pos_px = (u, v, area)

                if self.depth_img is not None:
                    try:
                        h, w = self.depth_img.shape[:2]
                        u_min = max(0, u - 2)
                        u_max = min(w, u + 3)
                        v_min = max(0, v - 2)
                        v_max = min(h, v + 3)

                        roi = self.depth_img[v_min:v_max, u_min:u_max]
                        valid = roi[np.isfinite(roi) & (roi > 0)]

                        if len(valid) == 0:
                            return

                        z = float(np.median(valid))

                    except Exception as e:
                        return

                    x_cam = (u - self.cx) * z / self.fx
                    y_cam = (v - self.cy) * z / self.fy
                    z_cam = z

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'zed_camera_center'
                    t.child_frame_id = 'neon_bottle'
                    t.transform.translation.x = x_cam
                    t.transform.translation.y = y_cam
                    t.transform.translation.z = z_cam
                    t.transform.rotation.w = 1.0
                    self.tf_broadcaster.sendTransform(t)

                    # Publish numeric position to /bottle_position
                    point_msg = Point()
                    point_msg.x = x_cam
                    point_msg.y = y_cam
                    point_msg.z = z_cam
                    self.bottle_pub.publish(point_msg)

                    self.publish_visual_marker(x_cam, y_cam, z_cam, u, v, c, cv_img, is_valid=True)

    def depth_cb(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


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
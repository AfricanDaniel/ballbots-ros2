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


class TennisBallTracker(Node):
    def __init__(self):
        super().__init__('tennis_ball_tracker')

        # ============================================================
        # EASY VALIDATION TOGGLE - CHANGE THIS FOR TESTING!
        # ============================================================
        # Set to False to accept ALL green blobs (no validation)
        # Set to True to validate shape, size, depth consistency
        self.ENABLE_VALIDATION = True  # <-- CHANGE THIS LINE
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

        # Publisher for ball location
        self.ball_pub = self.create_publisher(Point, '/tennis_ball_position', 10)

        # Publisher for visual marker in RViz
        self.marker_pub = self.create_publisher(Marker, '/tennis_ball_marker', 10)

        # Publisher for annotated image with bounding box
        self.annotated_img_pub = self.create_publisher(Image, '/tennis_ball_detection_image', 10)

        # Publisher for ALL detections (including rejected ones)
        self.debug_img_pub = self.create_publisher(Image, '/tennis_ball_debug_image', 10)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.depth_img = None
        self.ball_pos_px = None  # (u, v, area)

        # TF broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Odom origin (set at first detection)
        self.odom_set = False
        self.odom_origin = None  # (x, y, z)

        # Motion tracking for pattern analysis
        self.position_history = deque(maxlen=10)
        self.last_position = None
        self.last_timestamp = None

        # Validation parameters (only used if ENABLE_VALIDATION = True)
        self.CIRCULARITY_THRESHOLD = 0.05
        self.ASPECT_RATIO_THRESHOLD = 0.2
        self.MIN_DIAMETER_M = 0.025
        self.MAX_DIAMETER_M = 0.15
        self.DEPTH_STD_THRESHOLD = 0.7
        self.MAX_VELOCITY_MPS = 10.0
        self.MAX_DETECTION_DISTANCE_M = 15.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("TennisBallTracker node started")
        self.get_logger().info(f"VALIDATION ENABLED: {self.ENABLE_VALIDATION}")
        if not self.ENABLE_VALIDATION:
            self.get_logger().warn("⚠️  VALIDATION DISABLED - ALL GREEN BLOBS ACCEPTED")
        self.get_logger().info("=" * 60)

    def cam_info_cb(self, msg: CameraInfo):
        """Get camera intrinsics from CameraInfo message"""
        if self.cam_info_received:
            return

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.cam_info_received = True
        self.get_logger().info(
            f"Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def is_tennis_ball(self, contour, u, v, depth_img):
        """
        Verify if detected blob is actually a tennis ball.
        Checks: shape, roundness, physical size, depth consistency.
        Returns: (is_valid, debug_info)
        """
        debug_info = {}

        # 1. Check roundness/circularity
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            debug_info['fail_reason'] = 'zero_perimeter'
            return False, debug_info

        area = cv2.contourArea(contour)
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        debug_info['circularity'] = circularity

        # 2. Check if shape is approximately circular using ellipse fitting
        aspect_ratio = 0
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
            (x_e, y_e), (MA, ma), angle = ellipse
            aspect_ratio = min(MA, ma) / max(MA, ma) if max(MA, ma) > 0 else 0
            debug_info['aspect_ratio'] = aspect_ratio

        # 3. Check physical size using depth
        try:
            h, w = depth_img.shape[:2]

            u_min = max(0, u - 2)
            u_max = min(w, u + 3)
            v_min = max(0, v - 2)
            v_max = min(h, v + 3)

            roi = depth_img[v_min:v_max, u_min:u_max]
            valid = roi[np.isfinite(roi) & (roi > 0)]

            if len(valid) == 0:
                debug_info['fail_reason'] = 'no_valid_depth'
                return False, debug_info

            z = float(np.median(valid))

            # Adaptive thresholds based on distance
            if z < 0.25:
                circularity_threshold = 0.02
                aspect_threshold = 0.10
                min_diameter = 0.015
            else:
                circularity_threshold = self.CIRCULARITY_THRESHOLD
                aspect_threshold = self.ASPECT_RATIO_THRESHOLD
                min_diameter = self.MIN_DIAMETER_M

            if not np.isfinite(z) or z <= 0.0 or z > self.MAX_DETECTION_DISTANCE_M:
                debug_info['fail_reason'] = f'invalid_depth (z={z:.2f}m, max={self.MAX_DETECTION_DISTANCE_M}m)'
                if self.debug_logs:
                    self.get_logger().info(f"❌ REJECTED: {debug_info['fail_reason']}")
                return False, debug_info

            # Estimate physical diameter from pixel area
            pixel_radius = np.sqrt(area / np.pi)
            physical_diameter = 2 * pixel_radius * z / self.fx
            debug_info['estimated_diameter_cm'] = physical_diameter * 100
            debug_info['depth_m'] = z

            if circularity < circularity_threshold:
                debug_info['fail_reason'] = f'low_circularity ({circularity:.2f} < {circularity_threshold})'
                return False, debug_info

            if aspect_ratio < aspect_threshold:
                debug_info['fail_reason'] = f'not_circular (aspect={aspect_ratio:.2f} < {aspect_threshold})'
                return False, debug_info

            if physical_diameter < min_diameter or physical_diameter > self.MAX_DIAMETER_M:
                debug_info['fail_reason'] = (
                    f'wrong_size ({physical_diameter * 100:.1f}cm, '
                    f'range: {min_diameter * 100:.1f}-{self.MAX_DIAMETER_M * 100:.1f}cm)'
                )
                return False, debug_info

            # 4. Check depth consistency across detected region
            x, y, w, h = cv2.boundingRect(contour)
            roi_depth = depth_img[max(0, y):min(depth_img.shape[0], y + h),
                                  max(0, x):min(depth_img.shape[1], x + w)]

            valid_depths = roi_depth[roi_depth > 0]
            if len(valid_depths) > 0:
                depth_std = np.std(valid_depths)
                debug_info['depth_std'] = depth_std

                if depth_std > self.DEPTH_STD_THRESHOLD:
                    debug_info['fail_reason'] = (
                        f'inconsistent_depth (std={depth_std:.3f}m > {self.DEPTH_STD_THRESHOLD}m)'
                    )
                    if self.debug_logs:
                        self.get_logger().info(f"❌ REJECTED: {debug_info['fail_reason']}")
                    return False, debug_info

        except Exception as e:
            debug_info['fail_reason'] = f'exception: {e}'
            self.get_logger().warn(f"❌ REJECTED: {debug_info['fail_reason']}")
            return False, debug_info

        debug_info['pass'] = True
        if self.debug_logs:
            self.get_logger().info(
                f"✅ ACCEPTED: circularity={circularity:.2f}, aspect={debug_info.get('aspect_ratio', 0):.2f}, "
                f"diameter={physical_diameter * 100:.1f}cm, depth={z:.2f}m, "
                f"depth_std={debug_info.get('depth_std', 0):.3f}m")
        return True, debug_info

    def check_motion_pattern(self, current_pos):
        """Check motion pattern to verify it's a real object"""
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
        """Publish visual marker for RViz and annotated image"""

        # 1. Publish 3D marker for RViz (only if valid)
        if is_valid:
            marker = Marker()
            marker.header.frame_id = 'zed_camera_center'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'tennis_ball'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x_cam
            marker.pose.position.y = y_cam
            marker.pose.position.z = z_cam
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.067
            marker.scale.y = 0.067
            marker.scale.z = 0.067
            marker.color = ColorRGBA(r=0.8, g=1.0, b=0.0, a=0.8)
            marker.lifetime.sec = 1
            self.marker_pub.publish(marker)

        # 2. Publish annotated image
        annotated_img = cv_img.copy()

        if is_valid:
            contour_color = (0, 255, 0)
            bbox_color = (0, 255, 255)
            center_color = (0, 0, 255)
            label = f'Tennis Ball ({z_cam:.1f}m)'
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

        # Always publish to debug topic (shows ALL detections including rejected)
        self.debug_img_pub.publish(annotated_msg)

    def rgb_cb(self, msg: Image):
        """Process RGB image to detect tennis ball"""
        if None in [self.fx, self.fy, self.cx, self.cy]:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (29, 86, 6), (64, 255, 255))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < 10:
                return

            M = cv2.moments(c)
            if M['m00'] > 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])

                is_valid = True
                debug_info = {}

                if self.ENABLE_VALIDATION and self.depth_img is not None:
                    is_valid, debug_info = self.is_tennis_ball(c, u, v, self.depth_img)

                if not self.ENABLE_VALIDATION or is_valid:
                    self.ball_pos_px = (u, v, area)

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
                            self.get_logger().warn(f"Depth read failed: {e}")
                            return

                        x_cam = (u - self.cx) * z / self.fx
                        y_cam = (v - self.cy) * z / self.fy
                        z_cam = z

                        motion_valid, motion_info = self.check_motion_pattern((x_cam, y_cam, z_cam))
                        if 'motion_warning' in motion_info:
                            self.get_logger().warn(f"Motion: {motion_info['motion_warning']}")

                        # Initialize odom origin at first detection
                        if not self.odom_set:
                            self.odom_origin = (x_cam, y_cam, z_cam)
                            self.odom_set = True

                            t_static = TransformStamped()
                            t_static.header.stamp = self.get_clock().now().to_msg()
                            t_static.header.frame_id = 'odom'
                            t_static.child_frame_id = 'zed_camera_center'
                            t_static.transform.translation.x = self.odom_origin[0]
                            t_static.transform.translation.y = self.odom_origin[1]
                            t_static.transform.translation.z = self.odom_origin[2]
                            t_static.transform.rotation.x = 0.0
                            t_static.transform.rotation.y = 0.0
                            t_static.transform.rotation.z = 0.0
                            t_static.transform.rotation.w = 1.0
                            self.static_tf_broadcaster.sendTransform(t_static)

                        x_rel = x_cam - self.odom_origin[0]
                        y_rel = y_cam - self.odom_origin[1]
                        z_rel = z_cam - self.odom_origin[2]

                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = 'zed_camera_center'
                        t.child_frame_id = 'tennis_ball'
                        t.transform.translation.x = x_rel
                        t.transform.translation.y = y_rel
                        t.transform.translation.z = z_rel
                        t.transform.rotation.x = 0.0
                        t.transform.rotation.y = 0.0
                        t.transform.rotation.z = 0.0
                        t.transform.rotation.w = 1.0
                        self.tf_broadcaster.sendTransform(t)

                        # Publish numeric position
                        point_msg = Point()
                        point_msg.x = x_cam
                        point_msg.y = y_cam
                        point_msg.z = z_cam
                        self.ball_pub.publish(point_msg)

                        self.publish_visual_marker(x_cam, y_cam, z_cam, u, v, c, cv_img, is_valid=True)

                else:
                    # Validation failed - publish debug image to show what was rejected
                    if self.depth_img is not None:
                        try:
                            z = float(self.depth_img[v, u])
                            if z > 0.0:
                                x_cam = (u - self.cx) * z / self.fx
                                y_cam = (v - self.cy) * z / self.fy
                                z_cam = z
                                self.publish_visual_marker(x_cam, y_cam, z_cam, u, v, c, cv_img, is_valid=False)
                        except Exception:
                            pass

    def depth_cb(self, msg: Image):
        """Save latest depth image"""
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def print_ball_info(self):
        """Print ball info every 5 seconds"""
        if self.ball_pos_px and self.depth_img is not None:
            u, v, area = self.ball_pos_px
            try:
                z = float(self.depth_img[v, u])
                if z > 0.0:
                    print(f"Tennis ball area: {area:.1f} px, approx. distance: {z:.2f} m")
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = TennisBallTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
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
        # EASY VALIDATION TOGGLE - CHANGE THIS FOR TESTING!
        # ============================================================
        self.ENABLE_VALIDATION = True
        # ============================================================

        # ============================================================
        # POLE DETECTION CONFIG
        # Tune HSV range to match your specific dark green pole
        # under your court lighting conditions
        # ============================================================
        # self.POLE_H_LOW = 100  # blue
        # self.POLE_H_HIGH = 130
        # self.POLE_S_LOW = 80
        # self.POLE_S_HIGH = 255
        # self.POLE_V_LOW = 80
        # self.POLE_V_HIGH = 255
        # self.POLE_MIN_AREA = 300  # bottle is bigger than a thin pole
        self.POLE_H_LOW = 5  # orange hue
        self.POLE_H_HIGH = 20
        self.POLE_S_LOW = 150  # orange is very saturated
        self.POLE_S_HIGH = 255
        self.POLE_V_LOW = 100  # bright
        self.POLE_V_HIGH = 255
        self.POLE_MIN_AREA = 300
        self.POLE_ASPECT_RATIO = 0.5  # bottle is roughly as tall as wide, relax this
        self.POLE_CONFIRM_FRAMES = 3     # consecutive frames before publishing
        # ============================================================

        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub     = self.create_subscription(Image,      '/zed/zed_node/rgb/color/rect/image',       self.rgb_cb,      10)
        self.depth_sub   = self.create_subscription(Image,      '/zed/zed_node/depth/depth_registered',     self.depth_cb,    10)
        self.cam_info_sub= self.create_subscription(CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info', self.cam_info_cb, 10)

        # Publishers — bottle
        self.bottle_pub    = self.create_publisher(Point,  '/bottle_position',    10)
        self.marker_pub    = self.create_publisher(Marker, '/bottle_marker',      10)
        self.debug_img_pub = self.create_publisher(Image,  '/bottle_debug_image', 10)

        # Publishers — pole
        self.pole_pub         = self.create_publisher(Point, '/court_pole_position', 10)
        self.pole_debug_pub   = self.create_publisher(Image, '/pole_debug_image',    10)

        self.fx = self.fy = self.cx = self.cy = None
        self.depth_img = None
        self.cam_info_received = False
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.frame_count = 0
        self._pole_seen_count = 0  # consecutive frames pole was detected

        self.get_logger().info("=" * 60)
        self.get_logger().info("Bottle + Court Pole Tracker started!")
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

        cv_img   = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        debug_img = cv_img.copy()
        h, w     = cv_img.shape[:2]
        hsv      = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # ── HSV profiler ──────────────────────────────────────────────
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            margin = 10
            sample = hsv[h // 2 - margin:h // 2 + margin, w // 2 - margin:w // 2 + margin]
            valid  = sample[sample[:, :, 1] > 20]
            if len(valid) > 0:
                self.get_logger().info(
                    f"Center 20x20 HSV median: H={int(np.median(valid[:,0]))}, "
                    f"S={int(np.median(valid[:,1]))}, V={int(np.median(valid[:,2]))}")
            else:
                c = hsv[h // 2, w // 2]
                self.get_logger().info(f"Center pixel HSV: H={c[0]}, S={c[1]}, V={c[2]}")

        cv2.circle(debug_img, (w // 2, h // 2), 4, (255, 255, 255), -1)
        cv2.circle(debug_img, (w // 2, h // 2), 2, (0,   0,   0),   -1)

        # ── Bottle HSV mask ───────────────────────────────────────────
        self.INDOOR_MODE = False
        self.SALMON_MODE = False
        self.YELLOW_MODE = False
        self.BLUE_MODE   = True

        if self.YELLOW_MODE:
            mask = cv2.inRange(hsv, (20,  80,  80), (35,  255, 255))
        elif self.BLUE_MODE:
            mask = cv2.inRange(hsv, (100, 80,  80), (130, 255, 255))
        elif self.SALMON_MODE:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, (0,   30, 60), (20,  255, 255)),
                cv2.inRange(hsv, (160, 30, 60), (179, 255, 255)))
        elif self.INDOOR_MODE:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, (0,   90, 80), (12,  255, 255)),
                cv2.inRange(hsv, (168, 90, 80), (179, 255, 255)))
        else:
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, (0,   60, 60), (15,  255, 255)),
                cv2.inRange(hsv, (165, 60, 60), (179, 255, 255)))

        # ── Bottle detection ──────────────────────────────────────────
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bottle_found  = False
        best_contour  = None

        if contours:
            if not self.ENABLE_VALIDATION:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) >= 50:
                    best_contour = c
                    cv2.putText(debug_img, 'VALIDATION OFF', (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            else:
                valid_contours = []
                for c in contours:
                    area = cv2.contourArea(c)
                    if area < 150: continue
                    x_b, y_b, w_b, h_b = cv2.boundingRect(c)
                    if w_b == 0: continue
                    aspect_ratio = float(h_b) / float(w_b)

                    rough_z = 0.0
                    if self.depth_img is not None:
                        try:
                            dh, dw = self.depth_img.shape[:2]
                            bcy = min(y_b + h_b // 2, dh - 1)
                            bcx = min(x_b + w_b // 2, dw - 1)
                            rough_z = float(self.depth_img[bcy, bcx])
                        except Exception:
                            pass

                    if rough_z > 3000 or rough_z <= 0:
                        min_r, max_r = 0.4, 8.0
                    elif rough_z > 1500:
                        min_r, max_r = 0.8, 6.0
                    else:
                        min_r, max_r = 0.8, 6.0

                    if min_r <= aspect_ratio <= max_r:
                        valid_contours.append(c)
                    else:
                        cv2.rectangle(debug_img, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 0, 255), 2)
                        cv2.putText(debug_img, f'REJECTED ({aspect_ratio:.1f})',
                                    (x_b, y_b - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                if valid_contours:
                    best_contour = max(valid_contours, key=cv2.contourArea)

        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M['m00'] > 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                cv2.drawContours(debug_img, [best_contour], -1, (255, 0, 255), 2)
                x_b, y_b, w_b, h_b = cv2.boundingRect(best_contour)
                cv2.rectangle(debug_img, (x_b, y_b), (x_b + w_b, y_b + h_b), (255, 0, 255), 2)
                cv2.circle(debug_img, (u, v), 5, (0, 255, 0), -1)

                if self.depth_img is not None:
                    try:
                        dh, dw = self.depth_img.shape[:2]
                        u_min, u_max = max(0, u - 2), min(dw, u + 3)
                        v_min, v_max = max(0, v - 2), min(dh, v + 3)
                        roi   = self.depth_img[v_min:v_max, u_min:u_max]
                        valid = roi[np.isfinite(roi) & (roi > 0)]

                        if len(valid) > 0:
                            z     = float(np.median(valid))
                            x_cam = (u - self.cx) * z / self.fx
                            y_cam = (v - self.cy) * z / self.fy
                            bottle_found = True

                            pt = Point()
                            pt.x, pt.y, pt.z = x_cam, y_cam, z
                            self.bottle_pub.publish(pt)

                            cv2.putText(debug_img, f'Bottle ({z/1000:.2f}m)', (x_b, y_b - 25),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

                            t = TransformStamped()
                            t.header.stamp    = self.get_clock().now().to_msg()
                            t.header.frame_id = 'zed_camera_center'
                            t.child_frame_id  = 'neon_bottle'
                            t.transform.translation.x = float(x_cam)
                            t.transform.translation.y = float(y_cam)
                            t.transform.translation.z = float(z)
                            t.transform.rotation.w    = 1.0
                            self.tf_broadcaster.sendTransform(t)

                            marker = Marker()
                            marker.header.frame_id = 'zed_camera_center'
                            marker.header.stamp    = self.get_clock().now().to_msg()
                            marker.ns   = 'pink_bottle'
                            marker.id   = 1
                            marker.type = Marker.CYLINDER
                            marker.action = Marker.ADD
                            marker.pose.position.x = x_cam
                            marker.pose.position.y = y_cam
                            marker.pose.position.z = z
                            marker.pose.orientation.w = 1.0
                            marker.scale.x = 0.08
                            marker.scale.y = 0.08
                            marker.scale.z = 0.20
                            marker.color   = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
                            marker.lifetime.sec = 1
                            self.marker_pub.publish(marker)
                        else:
                            cv2.putText(debug_img, 'No Depth', (x_b, y_b - 25),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    except Exception:
                        pass

        if not bottle_found:
            cv2.putText(debug_img, 'Searching for Bottle...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)

        annotated = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        annotated.header.stamp    = self.get_clock().now().to_msg()
        annotated.header.frame_id = 'zed_camera_center'
        self.debug_img_pub.publish(annotated)

        # ── POLE DETECTION ────────────────────────────────────────────
        self._detect_pole(cv_img, hsv, msg)

    # =================================================================
    # POLE DETECTION
    # =================================================================
    def _detect_pole(self, cv_img, hsv, original_msg):
        """Detect the dark green tennis net pole. Always publishes debug image."""
        pole_debug = cv_img.copy()
        h, w = cv_img.shape[:2]

        # Dark green mask
        pole_mask = cv2.inRange(
            hsv,
            (self.POLE_H_LOW,  self.POLE_S_LOW,  self.POLE_V_LOW),
            (self.POLE_H_HIGH, self.POLE_S_HIGH,  self.POLE_V_HIGH)
        )

        # Morphology — fill gaps in pole, remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
        pole_mask = cv2.morphologyEx(pole_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        pole_mask = cv2.morphologyEx(pole_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        # Draw mask overlay on debug (green tint where mask fires)
        mask_overlay = cv2.cvtColor(pole_mask, cv2.COLOR_GRAY2BGR)
        mask_overlay[:, :, 0] = 0  # zero out B
        mask_overlay[:, :, 2] = 0  # zero out R — keep only G channel
        pole_debug = cv2.addWeighted(pole_debug, 0.7, mask_overlay, 0.3, 0)

        contours, _ = cv2.findContours(pole_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pole_found    = False
        best_pole     = None
        best_pole_cx  = 0

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.POLE_MIN_AREA:
                continue
            x_b, y_b, w_b, h_b = cv2.boundingRect(c)
            if w_b == 0: continue
            aspect = float(h_b) / float(w_b)

            # Draw all candidates in yellow
            cv2.rectangle(pole_debug, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 255, 255), 1)
            cv2.putText(pole_debug, f'ar={aspect:.1f} a={int(area)}',
                        (x_b, y_b - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            if aspect >= self.POLE_ASPECT_RATIO:
                if best_pole is None or area > cv2.contourArea(best_pole):
                    best_pole    = c
                    best_pole_cx = x_b + w_b // 2

        if best_pole is not None:
            self._pole_seen_count += 1
            x_b, y_b, w_b, h_b = cv2.boundingRect(best_pole)
            cx = x_b + w_b // 2

            # Draw confirmed pole in bright green
            cv2.rectangle(pole_debug, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 255, 0), 3)
            cv2.drawContours(pole_debug, [best_pole], -1, (0, 255, 0), 2)
            cv2.putText(pole_debug, f'POLE ({self._pole_seen_count}/{self.POLE_CONFIRM_FRAMES})',
                        (x_b, y_b - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Normalised x offset: -1.0 (far left) to +1.0 (far right)
            x_norm = (cx - w / 2.0) / (w / 2.0)

            if self._pole_seen_count >= self.POLE_CONFIRM_FRAMES:
                pt = Point()
                pt.x = x_norm
                pt.y = 0.0
                pt.z = 1.0   # placeholder — no depth needed, just direction
                self.pole_pub.publish(pt)
                pole_found = True
                cv2.putText(pole_debug, '✓ PUBLISHED', (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            self._pole_seen_count = 0

        # Status text
        if not pole_found:
            label = f'Searching for pole... (seen={self._pole_seen_count})'
            cv2.putText(pole_debug, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # HSV range label so you know what you're tuning
        cv2.putText(pole_debug,
                    f'H:{self.POLE_H_LOW}-{self.POLE_H_HIGH} '
                    f'S:{self.POLE_S_LOW}-{self.POLE_S_HIGH} '
                    f'V:{self.POLE_V_LOW}-{self.POLE_V_HIGH}',
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Always publish pole debug image
        pole_debug_msg = self.bridge.cv2_to_imgmsg(pole_debug, encoding='bgr8')
        pole_debug_msg.header.stamp    = self.get_clock().now().to_msg()
        pole_debug_msg.header.frame_id = 'zed_camera_center'
        self.pole_debug_pub.publish(pole_debug_msg)


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
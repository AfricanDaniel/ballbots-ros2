#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import tf2_ros
import cv2
import numpy as np


class RealSenseTracker(Node):
    def __init__(self):
        super().__init__('realsense_tracker')

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_img = None

        # =====================================================
        # BALL TUNING CONSTANTS
        # =====================================================
        self.HSV_LOWER = (22, 60, 60)
        self.HSV_UPPER = (48, 255, 255)
        self.MIN_AREA  = 50
        self.GRAB_AREA = 175000
        self.DEPTH_ROI = 3

        # =====================================================
        # FLOOR REMOVAL CONSTANTS
        # =====================================================
        self.ENABLE_FLOOR_REMOVAL = False
        self.CAMERA_HEIGHT_M      = 0.3
        self.CAMERA_TILT_DEG      = 0.0
        self.FLOOR_MARGIN_M       = 0.0

        # =====================================================
        # BOTTLE (BLUE) DETECTION
        # Tightened for purple courts:
        #   - Hue  100-118  →  pure blue only (court purple sits at 120-145)
        #   - Sat  140+     →  vivid bottle colour (court purple is dull, S~40-80)
        #   - Val  80+      →  ignore very dark shadows
        # If bottle is missed: lower S to 120 first, then nudge H_HIGH up to 120.
        # If court bleeds in: raise S_LOW to 160, or drop H_HIGH to 115.
        # =====================================================
        self.BOTTLE_H_LOW  = 100  #blue
        self.BOTTLE_H_HIGH = 118
        self.BOTTLE_S_LOW  = 140
        # self.BOTTLE_H_LOW = 20  # yellow hue range
        # self.BOTTLE_H_HIGH = 35
        # self.BOTTLE_S_LOW = 100  # keep saturation strict to avoid court bleed
        self.BOTTLE_S_HIGH = 255
        self.BOTTLE_V_LOW  = 80
        self.BOTTLE_V_HIGH = 255
        self.BOTTLE_MIN_AREA          = 100
        self.BOTTLE_ENABLE_VALIDATION = True   # aspect ratio filter
        self.BOTTLE_CONFIRM_FRAMES    = 3      # consecutive frames before publishing

        # =====================================================
        # POLE (RED) DETECTION
        # =====================================================
        self.POLE_H_LOW   = 0       # red lower hue range
        self.POLE_H_HIGH  = 10
        self.POLE_H_LOW2  = 165     # red upper hue range (wraps at 180)
        self.POLE_H_HIGH2 = 179
        self.POLE_S_LOW   = 120
        self.POLE_S_HIGH  = 255
        self.POLE_V_LOW   = 80
        self.POLE_V_HIGH  = 255
        self.POLE_MIN_AREA     = 200
        self.POLE_ASPECT_RATIO = 1.5   # height/width — tall narrow pole
        self.POLE_CONFIRM_FRAMES = 3

        # Internal counters
        self._bottle_seen_count = 0
        self._pole_seen_count   = 0

        # =====================================================
        # PARAMETERS
        # =====================================================
        self.declare_parameter('debug_logs', False)
        self.debug_logs = self.get_parameter('debug_logs').value

        # =====================================================
        # SUBSCRIBERS
        # =====================================================
        self.create_subscription(CameraInfo,
            '/camera/camera/color/camera_info', self.cam_info_cb, 10)
        self.create_subscription(Image,
            '/camera/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image,
            '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)

        # =====================================================
        # PUBLISHERS — ball
        # =====================================================
        self.pub       = self.create_publisher(Point,           '/tennis_ball_position_close', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/realsense/debug_image/compressed', 10)

        # Publishers — bottle
        self.bottle_pub        = self.create_publisher(Point,           '/bottle_position',                   10)
        self.bottle_marker_pub = self.create_publisher(Marker,          '/bottle_marker',                     10)
        self.bottle_debug_pub  = self.create_publisher(CompressedImage, '/realsense/bottle_debug/compressed', 10)

        # Publishers — pole
        self.pole_pub       = self.create_publisher(Point,           '/court_pole_position',            10)
        self.pole_debug_pub = self.create_publisher(CompressedImage, '/realsense/pole_debug/compressed', 10)

        # TF broadcaster (for neon_bottle frame used by ball_chaser)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("=" * 60)
        self.get_logger().info("RealSense tracker ready  (ball + bottle + pole)")
        self.get_logger().info(
            f"Floor removal: {'ON' if self.ENABLE_FLOOR_REMOVAL else 'OFF'} "
            f"(height={self.CAMERA_HEIGHT_M*100:.0f}cm, "
            f"tilt={self.CAMERA_TILT_DEG}deg, "
            f"margin={self.FLOOR_MARGIN_M*100:.0f}cm)"
        )
        self.get_logger().info("=" * 60)

    # =========================================================
    # CAMERA CALLBACKS
    # =========================================================
    def cam_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f"Camera intrinsics loaded: fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f}"
            )

    def depth_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # =========================================================
    # FLOOR REMOVAL
    # =========================================================
    def _remove_floor(self, cv_img):
        if self.depth_img is None:
            return cv_img

        h_img, w_img = cv_img.shape[:2]

        if self.depth_img.shape[:2] != (h_img, w_img):
            depth_resized = cv2.resize(
                self.depth_img, (w_img, h_img), interpolation=cv2.INTER_NEAREST)
        else:
            depth_resized = self.depth_img.copy()

        tilt_rad = np.radians(self.CAMERA_TILT_DEG)
        Z = np.nan_to_num(depth_resized.astype(float), nan=0.0, posinf=0.0, neginf=0.0)
        Z /= 1000.0  # mm -> m

        v_grid, _ = np.mgrid[0:h_img, 0:w_img]
        Y_cam   = (v_grid - self.cy) * Z / self.fy
        Y_floor = (self.CAMERA_HEIGHT_M * np.cos(tilt_rad)) - (Z * np.sin(tilt_rad))

        is_valid_depth = (Z > 0.05) & (Z < 15.0)
        is_above_floor = Y_cam < (Y_floor - self.FLOOR_MARGIN_M)
        horizon_mask   = np.ones((h_img, w_img), dtype=bool)  # keep all rows

        mask_keep = (is_valid_depth & is_above_floor) | horizon_mask
        result = cv_img.copy()
        result[~mask_keep] = [0, 0, 0]
        return result

    # =========================================================
    # DEPTH HELPER — returns z in METRES from RealSense mm depth
    # =========================================================
    def _get_depth_m(self, u, v, roi_r=3):
        if self.depth_img is None:
            return float('nan')
        h, w = self.depth_img.shape[:2]
        roi = self.depth_img[
            max(0, v - roi_r):min(h, v + roi_r + 1),
            max(0, u - roi_r):min(w, u + roi_r + 1)
        ].astype(float)
        roi[roi == 0] = np.nan
        z_mm = float(np.nanmedian(roi))
        return z_mm / 1000.0 if not np.isnan(z_mm) and z_mm > 0 else float('nan')

    # =========================================================
    # MAIN RGB CALLBACK — runs all three detectors
    # =========================================================
    def rgb_cb(self, msg):
        if None in [self.fx, self.fy, self.cx, self.cy] or self.depth_img is None:
            self.get_logger().info(
                "Waiting: camera_info or depth not ready", throttle_duration_sec=2.0)
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Floor removal only for ball (keeps close-to-floor objects visible for bottle/pole)
        cv_img_clean = self._remove_floor(cv_img) if self.ENABLE_FLOOR_REMOVAL else cv_img

        hsv_clean = cv2.cvtColor(cv_img_clean, cv2.COLOR_BGR2HSV)
        hsv_raw   = cv2.cvtColor(cv_img,       cv2.COLOR_BGR2HSV)

        self._detect_ball(cv_img_clean, hsv_clean)
        self._detect_bottle(cv_img, hsv_raw)
        self._detect_pole(cv_img, hsv_raw)

    # =========================================================
    # BALL DETECTION (yellow-green tennis ball)
    # =========================================================
    def _detect_ball(self, cv_img, hsv):
        h_img, w_img = cv_img.shape[:2]
        total_pixels = h_img * w_img

        mask      = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)
        hsv_pct   = (cv2.countNonZero(mask) / total_pixels) * 100.0

        dh, dw    = self.depth_img.shape[:2]
        margin    = 80
        c_depth   = self.depth_img[
            max(0, dh//2 - margin):min(dh, dh//2 + margin),
            max(0, dw//2 - margin):min(dw, dw//2 + margin)]
        nan_pct   = int(np.sum((c_depth == 0) | np.isnan(c_depth))) / c_depth.size * 100.0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().info("[RS] No ball contours", throttle_duration_sec=1.0)
            self._publish_compressed(self.debug_pub, cv_img)
            return

        c    = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        x_b, y_b, w_b, h_b = cv2.boundingRect(c)
        perim = cv2.arcLength(c, True)
        circ  = (4 * np.pi * area / (perim * perim)) if perim > 0 else 0.0

        if area < self.MIN_AREA:
            self._publish_compressed(self.debug_pub, cv_img)
            return

        M = cv2.moments(c)
        if M['m00'] == 0:
            return
        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        z = self._get_depth_m(u, v, self.DEPTH_ROI)
        if np.isnan(z):
            if area >= self.GRAB_AREA:
                z = 0.15
            else:
                self._publish_debug_ball(cv_img, c, u, v, 0.0, area,
                                         nan_pct, hsv_pct, circ, w_b, h_b, valid=False)
                return

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy

        self.get_logger().info(
            f"[RS] Ball: z={z:.3f}m x={x_cam:.3f}m area={area:.0f}px",
            throttle_duration_sec=0.5)

        self._publish_debug_ball(cv_img, c, u, v, z, area,
                                 nan_pct, hsv_pct, circ, w_b, h_b, valid=True)
        pt = Point()
        pt.x, pt.y, pt.z = x_cam, y_cam, z
        self.pub.publish(pt)

    # =========================================================
    # BOTTLE DETECTION (blue)
    # =========================================================
    def _detect_bottle(self, cv_img, hsv):
        h_img, w_img = cv_img.shape[:2]
        total_pixels  = h_img * w_img

        mask    = cv2.inRange(hsv,
                              (self.BOTTLE_H_LOW,  self.BOTTLE_S_LOW,  self.BOTTLE_V_LOW),
                              (self.BOTTLE_H_HIGH, self.BOTTLE_S_HIGH, self.BOTTLE_V_HIGH))
        hsv_pct = cv2.countNonZero(mask) / total_pixels * 100.0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_img    = cv_img.copy()
        bottle_found = False
        best_contour = None
        z_out        = 0.0

        if contours:
            if not self.BOTTLE_ENABLE_VALIDATION:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) >= self.BOTTLE_MIN_AREA:
                    best_contour = c
            else:
                valid_cs = []
                for c in contours:
                    area = cv2.contourArea(c)
                    if area < self.BOTTLE_MIN_AREA:
                        continue
                    x_b, y_b, w_b, h_b = cv2.boundingRect(c)
                    if w_b == 0:
                        continue
                    aspect = float(h_b) / float(w_b)

                    rough_z_mm = 0.0
                    if self.depth_img is not None:
                        try:
                            rough_z_mm = float(self.depth_img[
                                min(y_b + h_b // 2, self.depth_img.shape[0] - 1),
                                min(x_b + w_b // 2, self.depth_img.shape[1] - 1)])
                        except Exception:
                            pass

                    if rough_z_mm > 3000 or rough_z_mm <= 0:
                        min_r, max_r = 0.3, 10.0
                    elif rough_z_mm > 1500:
                        min_r, max_r = 0.4, 8.0
                    else:
                        min_r, max_r = 0.5, 6.0

                    if min_r <= aspect <= max_r:
                        valid_cs.append(c)
                    else:
                        cv2.rectangle(debug_img, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 0, 255), 1)
                        cv2.putText(debug_img, f'ar={aspect:.1f}',
                                    (x_b, y_b - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                if valid_cs:
                    best_contour = max(valid_cs, key=cv2.contourArea)

        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M['m00'] > 0:
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])
                z = self._get_depth_m(u, v)

                if not np.isnan(z) and z > 0:
                    x_cam = (u - self.cx) * z / self.fx
                    y_cam = (v - self.cy) * z / self.fy
                    z_out = z

                    self._bottle_seen_count += 1

                    if self._bottle_seen_count >= self.BOTTLE_CONFIRM_FRAMES:
                        # Publish position
                        pt = Point()
                        pt.x, pt.y, pt.z = x_cam, y_cam, z
                        self.bottle_pub.publish(pt)
                        bottle_found = True

                        # TF — RealSense optical frame
                        t = TransformStamped()
                        t.header.stamp    = self.get_clock().now().to_msg()
                        t.header.frame_id = 'camera_color_optical_frame'
                        t.child_frame_id  = 'neon_bottle'
                        t.transform.translation.x = float(x_cam)
                        t.transform.translation.y = float(y_cam)
                        t.transform.translation.z = float(z)
                        t.transform.rotation.w    = 1.0
                        self.tf_broadcaster.sendTransform(t)

                        # RViz marker
                        marker = Marker()
                        marker.header.frame_id = 'camera_color_optical_frame'
                        marker.header.stamp    = self.get_clock().now().to_msg()
                        marker.ns     = 'blue_bottle'
                        marker.id     = 1
                        marker.type   = Marker.CYLINDER
                        marker.action = Marker.ADD
                        marker.pose.position.x = x_cam
                        marker.pose.position.y = y_cam
                        marker.pose.position.z = z
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 0.08
                        marker.scale.y = 0.08
                        marker.scale.z = 0.20
                        marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
                        marker.lifetime.sec = 1
                        self.bottle_marker_pub.publish(marker)

                    # Draw contour on debug
                    x_b, y_b, w_b, h_b = cv2.boundingRect(best_contour)
                    color = (255, 128, 0) if bottle_found else (0, 165, 255)
                    cv2.drawContours(debug_img, [best_contour], -1, color, 2)
                    cv2.rectangle(debug_img, (x_b, y_b), (x_b + w_b, y_b + h_b), color, 2)
                    cv2.circle(debug_img, (u, v), 5, (0, 255, 0), -1)
                    cv2.putText(debug_img,
                                f'Bottle {z:.2f}m  ({self._bottle_seen_count}/{self.BOTTLE_CONFIRM_FRAMES})',
                                (x_b, y_b - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                else:
                    self._bottle_seen_count = 0
        else:
            self._bottle_seen_count = 0

        # Stats overlay
        _area = cv2.contourArea(best_contour) if best_contour is not None else 0.0
        _ar   = 0.0
        w_b2  = h_b2 = 0
        if best_contour is not None:
            x_b2, y_b2, w_b2, h_b2 = cv2.boundingRect(best_contour)
            _ar = float(h_b2) / float(w_b2) if w_b2 > 0 else 0.0

        status_color = (255, 128, 0) if bottle_found else (0, 165, 255)
        cv2.putText(debug_img,
                    f"Bottle {z_out:.2f}m" if bottle_found else "Searching for Bottle...",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)
        for i, line in enumerate([
            f"area={_area:.0f}px  hsv={hsv_pct:.1f}%",
            f"bbox={w_b2}x{h_b2}  ar={_ar:.2f}",
            f"confirm={self._bottle_seen_count}/{self.BOTTLE_CONFIRM_FRAMES}",
            f"H:{self.BOTTLE_H_LOW}-{self.BOTTLE_H_HIGH} "
            f"S:{self.BOTTLE_S_LOW}-{self.BOTTLE_S_HIGH} "
            f"V:{self.BOTTLE_V_LOW}-{self.BOTTLE_V_HIGH}",
        ]):
            cv2.putText(debug_img, line, (10, 65 + i * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        self._publish_compressed(self.bottle_debug_pub, debug_img)

    # =========================================================
    # POLE DETECTION (red)
    # =========================================================
    def _detect_pole(self, cv_img, hsv):
        h_img, w_img = cv_img.shape[:2]
        pole_debug   = cv_img.copy()

        pole_mask = cv2.bitwise_or(
            cv2.inRange(hsv,
                        (self.POLE_H_LOW,  self.POLE_S_LOW, self.POLE_V_LOW),
                        (self.POLE_H_HIGH, self.POLE_S_HIGH, self.POLE_V_HIGH)),
            cv2.inRange(hsv,
                        (self.POLE_H_LOW2,  self.POLE_S_LOW, self.POLE_V_LOW),
                        (self.POLE_H_HIGH2, self.POLE_S_HIGH, self.POLE_V_HIGH))
        )

        kernel    = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7))
        pole_mask = cv2.morphologyEx(pole_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        pole_mask = cv2.morphologyEx(pole_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        # Green tint where mask fires
        overlay = cv2.cvtColor(pole_mask, cv2.COLOR_GRAY2BGR)
        overlay[:, :, 0] = 0
        overlay[:, :, 2] = 0
        pole_debug = cv2.addWeighted(pole_debug, 0.7, overlay, 0.3, 0)

        contours, _ = cv2.findContours(pole_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pole_found = False
        best_pole  = None

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.POLE_MIN_AREA:
                continue
            x_b, y_b, w_b, h_b = cv2.boundingRect(c)
            if w_b == 0:
                continue
            aspect = float(h_b) / float(w_b)

            cv2.rectangle(pole_debug, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 255, 255), 1)
            cv2.putText(pole_debug, f'ar={aspect:.1f} a={int(area)}',
                        (x_b, y_b - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            if aspect >= self.POLE_ASPECT_RATIO:
                if best_pole is None or area > cv2.contourArea(best_pole):
                    best_pole = c

        if best_pole is not None:
            self._pole_seen_count += 1
            x_b, y_b, w_b, h_b = cv2.boundingRect(best_pole)
            cx = x_b + w_b // 2
            x_norm = (cx - w_img / 2.0) / (w_img / 2.0)

            cv2.rectangle(pole_debug, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 255, 0), 3)
            cv2.drawContours(pole_debug, [best_pole], -1, (0, 255, 0), 2)
            cv2.putText(pole_debug,
                        f'POLE ({self._pole_seen_count}/{self.POLE_CONFIRM_FRAMES})',
                        (x_b, y_b - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if self._pole_seen_count >= self.POLE_CONFIRM_FRAMES:
                pt = Point()
                pt.x = x_norm
                pt.y = 0.0
                pt.z = 1.0   # direction only — no depth needed
                self.pole_pub.publish(pt)
                pole_found = True
                cv2.putText(pole_debug, 'PUBLISHED', (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            self._pole_seen_count = 0

        if not pole_found:
            cv2.putText(pole_debug,
                        f'Searching for pole... (seen={self._pole_seen_count})',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        cv2.putText(pole_debug,
                    f'H:{self.POLE_H_LOW}-{self.POLE_H_HIGH} | '
                    f'{self.POLE_H_LOW2}-{self.POLE_H_HIGH2}  '
                    f'S:{self.POLE_S_LOW}-{self.POLE_S_HIGH}  '
                    f'V:{self.POLE_V_LOW}-{self.POLE_V_HIGH}',
                    (10, h_img - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        self._publish_compressed(self.pole_debug_pub, pole_debug)

    # =========================================================
    # SHARED HELPERS
    # =========================================================
    def _publish_compressed(self, publisher, cv_img, quality=50):
        _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format       = "jpeg"
        msg.data         = buf.tobytes()
        publisher.publish(msg)

    def _publish_debug_ball(self, cv_img, contour, u, v, z, area,
                            nan_pct, hsv_pct, circularity, w_b, h_b, valid=True):
        debug_img = cv_img.copy()
        color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.drawContours(debug_img, [contour], -1, color, 2)
        x_b, y_b, _, _ = cv2.boundingRect(contour)
        cv2.rectangle(debug_img, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 255, 255), 2)
        cv2.circle(debug_img, (u, v), 5, (0, 0, 255), -1)

        floor_str = (f"FLOOR ON h={self.CAMERA_HEIGHT_M*100:.0f}cm "
                     f"margin={self.FLOOR_MARGIN_M*100:.0f}cm"
                     if self.ENABLE_FLOOR_REMOVAL else "FLOOR OFF")
        for i, line in enumerate([
            f"z={z:.3f}m  area={area:.0f}px",
            f"nan={nan_pct:.0f}%  hsv={hsv_pct:.1f}%",
            f"circ={circularity:.2f}  bbox={w_b}x{h_b}",
            f"centroid=({u},{v})",
            floor_str,
        ]):
            cv2.putText(debug_img, line, (10, 30 + i * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        if not valid:
            cv2.putText(debug_img, "SKIPPED", (10, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        self._publish_compressed(self.debug_pub, debug_img)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
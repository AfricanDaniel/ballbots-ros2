#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import cv2
import numpy as np


class RealSenseTracker(Node):
    def __init__(self):
        super().__init__('realsense_tracker')

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_img = None

        # =====================================================
        # TUNING CONSTANTS
        # =====================================================
        self.HSV_LOWER = (22, 60, 60)
        self.HSV_UPPER = (48, 255, 255)
        self.MIN_AREA  = 50
        self.GRAB_AREA = 175000
        self.DEPTH_ROI = 3

        # =====================================================
        # FLOOR REMOVAL CONSTANTS
        # Camera is mounted 4cm above ground, roughly level (no tilt)
        # Increase CAMERA_TILT_DEG if camera tilts upward
        # Increase FLOOR_MARGIN_M to be more aggressive
        # =====================================================
        self.ENABLE_FLOOR_REMOVAL = True   # ← toggle on/off
        self.CAMERA_HEIGHT_M      = 0.3   # 4cm above ground
        self.CAMERA_TILT_DEG      = 0.0    # adjust if camera is angled up/down
        self.FLOOR_MARGIN_M       = 0.0   # keep pixels >3cm above floor plane
        # =====================================================

        self.declare_parameter('debug_logs', False)
        self.debug_logs = self.get_parameter('debug_logs').value

        self.create_subscription(CameraInfo,
            '/camera/camera/color/camera_info', self.cam_info_cb, 10)
        self.create_subscription(Image,
            '/camera/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image,
            '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)

        self.pub       = self.create_publisher(Point,           '/tennis_ball_position_close', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/realsense/debug_image/compressed', 10)

        self.get_logger().info("RealSense tracker ready")
        self.get_logger().info(
            f"Floor removal: {'ON' if self.ENABLE_FLOOR_REMOVAL else 'OFF'} "
            f"(height={self.CAMERA_HEIGHT_M*100:.0f}cm, "
            f"tilt={self.CAMERA_TILT_DEG}deg, "
            f"margin={self.FLOOR_MARGIN_M*100:.0f}cm)"
        )

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

    def _remove_floor(self, cv_img):
        """
        Blacks out pixels that are at or below the floor plane.
        RealSense is mounted 4cm above ground, roughly horizontal.

        Y_cam at each pixel: Y_cam = (v - cy) * Z / fy
        Expected floor Y_cam at depth Z:
            Y_floor = CAMERA_HEIGHT * cos(tilt) - Z * sin(tilt)

        Keep pixel if Y_cam < Y_floor - FLOOR_MARGIN (i.e. above the floor)
        Note: in camera frame, larger Y = lower in world = closer to floor
        """
        if self.depth_img is None:
            return cv_img

        h_img, w_img = cv_img.shape[:2]

        # Resize depth to match RGB if dimensions differ
        if self.depth_img.shape[:2] != (h_img, w_img):
            depth_resized = cv2.resize(
                self.depth_img, (w_img, h_img),
                interpolation=cv2.INTER_NEAREST
            )
        else:
            depth_resized = self.depth_img.copy()

        tilt_rad = np.radians(self.CAMERA_TILT_DEG)

        # Z in meters (RealSense depth is mm)
        Z = np.nan_to_num(depth_resized.astype(float),
                          nan=0.0, posinf=0.0, neginf=0.0)
        Z /= 1000.0

        # Per-pixel Y in camera frame (downward = positive)
        v_grid, _ = np.mgrid[0:h_img, 0:w_img]
        Y_cam = (v_grid - self.cy) * Z / self.fy

        # Expected Y_cam of the floor at each distance Z
        Y_floor = (self.CAMERA_HEIGHT_M * np.cos(tilt_rad)) - (Z * np.sin(tilt_rad))

        # Keep only pixels that are above the floor by at least FLOOR_MARGIN
        is_valid_depth = (Z > 0.05) & (Z < 15.0)
        is_above_floor = Y_cam < (Y_floor - self.FLOOR_MARGIN_M)

        # Never mask the top 40% of the image — ball at far range is always up there
        horizon_row = int(h_img * 1.00)
        horizon_mask = np.zeros((h_img, w_img), dtype=bool)
        horizon_mask[:horizon_row, :] = True  # always keep top 40%

        mask_keep = (is_valid_depth & is_above_floor) | horizon_mask

        result = cv_img.copy()
        result[~mask_keep] = [0, 0, 0]
        return result

    def rgb_cb(self, msg):
        if None in [self.fx, self.fy, self.cx, self.cy] or self.depth_img is None:
            self.get_logger().info(
                "Waiting: camera_info or depth not ready", throttle_duration_sec=2.0)
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # =====================================================
        # FLOOR REMOVAL
        # =====================================================
        if self.ENABLE_FLOOR_REMOVAL:
            cv_img_clean = self._remove_floor(cv_img)
        else:
            cv_img_clean = cv_img
        # =====================================================

        hsv  = cv2.cvtColor(cv_img_clean, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)

        # Stats for debug overlay
        h_img, w_img = cv_img.shape[:2]
        total_pixels = h_img * w_img
        match_count  = cv2.countNonZero(mask)
        hsv_pct      = (match_count / total_pixels) * 100.0

        dh, dw   = self.depth_img.shape[:2]
        cdx, cdy = dw // 2, dh // 2
        margin   = 80
        center_depth = self.depth_img[
            cdy - margin:cdy + margin,
            cdx - margin:cdx + margin
        ]
        total_d      = center_depth.size
        invalid_d    = int(np.sum((center_depth == 0) | np.isnan(center_depth)))
        nan_pct      = (invalid_d / total_d) * 100.0
        valid_d_vals = center_depth[(center_depth > 0) & ~np.isnan(center_depth)]

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().info("[RS] No contours found", throttle_duration_sec=1.0)
            self._publish_debug_raw(cv_img_clean)
            return

        c    = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        x_b, y_b, w_b, h_b = cv2.boundingRect(c)
        perimeter   = cv2.arcLength(c, True)
        circularity = (4 * np.pi * area / (perimeter * perimeter)) if perimeter > 0 else 0.0

        if area < self.MIN_AREA:
            self._publish_debug_raw(cv_img_clean)
            return

        M = cv2.moments(c)
        if M['m00'] == 0:
            return

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        # Depth at centroid
        h, w = self.depth_img.shape[:2]
        r    = self.DEPTH_ROI
        roi  = self.depth_img[
            max(0, v - r):min(h, v + r + 1),
            max(0, u - r):min(w, u + r + 1)
        ].astype(float)
        roi[roi == 0] = np.nan
        z_mm = float(np.nanmedian(roi))

        if np.isnan(z_mm) or z_mm <= 0:
            if area >= self.GRAB_AREA:
                z = 0.15
            else:
                self._publish_debug_img(cv_img_clean, c, u, v, 0.0, area,
                                        nan_pct, hsv_pct, circularity, w_b, h_b,
                                        valid=False)
                return
        else:
            z = z_mm / 1000.0

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy

        self.get_logger().info(
            f"[RS] Ball: z={z:.3f}m x={x_cam:.3f}m area={area:.0f}px",
            throttle_duration_sec=0.5
        )

        self._publish_debug_img(cv_img_clean, c, u, v, z, area,
                                nan_pct, hsv_pct, circularity, w_b, h_b,
                                valid=True)

        pt = Point()
        pt.x = x_cam
        pt.y = y_cam
        pt.z = z
        self.pub.publish(pt)

    def _publish_debug_raw(self, cv_img):
        _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 50])
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.format = "jpeg"
        compressed_msg.data = buf.tobytes()
        self.debug_pub.publish(compressed_msg)

    def _publish_debug_img(self, cv_img, contour, u, v, z, area,
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
        lines = [
            f"z={z:.3f}m  area={area:.0f}px",
            f"nan={nan_pct:.0f}%  hsv={hsv_pct:.1f}%",
            f"circ={circularity:.2f}  bbox={w_b}x{h_b}",
            f"centroid=({u},{v})",
            floor_str,
        ]
        for i, line in enumerate(lines):
            cv2.putText(debug_img, line, (10, 30 + i * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        if not valid:
            cv2.putText(debug_img, "SKIPPED", (10, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.format = "jpeg"
        _, buf = cv2.imencode('.jpg', debug_img, [cv2.IMWRITE_JPEG_QUALITY, 50])
        compressed_msg.data = buf.tobytes()
        self.debug_pub.publish(compressed_msg)


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
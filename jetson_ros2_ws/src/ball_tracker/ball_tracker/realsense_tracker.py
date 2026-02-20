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
        self.HSV_LOWER = (25, 40, 40)  # was (35, 80, 80) — much more permissive
        self.HSV_UPPER = (80, 255, 255)  # was (65, 255, 255) — wider hue range
        self.MIN_AREA  = 50    # ignore blobs smaller than this
        self.GRAB_AREA = 175000   # area fallback threshold when depth is invalid 135000
        self.DEPTH_ROI = 3      # pixel radius for depth sampling at ball centroid
        # =====================================================

        # Parameters from launch file
        self.declare_parameter('debug_logs', False)
        self.debug_logs = self.get_parameter('debug_logs').value

        self.create_subscription(CameraInfo,
            '/camera/camera/color/camera_info', self.cam_info_cb, 10)
        self.create_subscription(Image,
            '/camera/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image,
            '/camera/camera/aligned_depth_to_color/image_raw', self.depth_cb, 10)

        self.pub       = self.create_publisher(Point,           '/tennis_ball_position_close', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/realsense/debug_image',      10)

        self.get_logger().info("RealSense tracker ready")

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

    def rgb_cb(self, msg):
        if None in [self.fx, self.fy, self.cx, self.cy] or self.depth_img is None:
            self.get_logger().info(
                "Waiting: camera_info or depth not ready", throttle_duration_sec=2.0)
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv    = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask   = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)

        # =====================================================
        # DATA COLLECTION — HSV pixel stats
        # =====================================================
        h_img, w_img = cv_img.shape[:2]
        total_pixels = h_img * w_img
        match_count  = cv2.countNonZero(mask)
        hsv_pct      = (match_count / total_pixels) * 100.0

        self.get_logger().info(
            f"[DATA] HSV={match_count}px ({hsv_pct:.1f}% of frame={w_img}x{h_img})",
            throttle_duration_sec=0.5
        )
        # =====================================================

        # =====================================================
        # DATA COLLECTION — Central depth region stats
        # 160x160 pixel region around image center.
        # High nan_pct means ball is in RealSense blind spot (<15cm)
        # =====================================================
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
        median_depth = float(np.median(valid_d_vals)) if len(valid_d_vals) > 0 else float('nan')

        self.get_logger().info(
            f"[DATA] CenterDepth: nan={nan_pct:.1f}% valid={100.0 - nan_pct:.1f}% "
            f"median={median_depth:.1f}mm",
            throttle_duration_sec=0.5
        )
        # =====================================================

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.get_logger().info("[DATA] No contours found", throttle_duration_sec=1.0)
            return

        c    = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        # =====================================================
        # DATA COLLECTION — Contour stats
        # =====================================================
        x_b, y_b, w_b, h_b = cv2.boundingRect(c)
        perimeter   = cv2.arcLength(c, True)
        circularity = (4 * np.pi * area / (perimeter * perimeter)) if perimeter > 0 else 0.0
        center_u    = x_b + w_b // 2
        center_v    = y_b + h_b // 2

        self.get_logger().info(
            f"[DATA] Contour: area={area:.0f}px bbox={w_b}x{h_b} "
            f"circ={circularity:.2f} center=({center_u},{center_v})",
            throttle_duration_sec=0.5
        )
        # =====================================================

        if area < self.MIN_AREA:
            self.get_logger().info(
                f"[DATA] area={area:.0f} < MIN_AREA={self.MIN_AREA} — skipping",
                throttle_duration_sec=1.0
            )
            return

        M = cv2.moments(c)
        if M['m00'] == 0:
            return

        u = int(M['m10'] / M['m00'])
        v = int(M['m01'] / M['m00'])

        # =====================================================
        # Depth at ball centroid (median over small ROI)
        # =====================================================
        h, w = self.depth_img.shape[:2]
        r    = self.DEPTH_ROI
        roi  = self.depth_img[
            max(0, v - r):min(h, v + r + 1),
            max(0, u - r):min(w, u + r + 1)
        ].astype(float)
        roi[roi == 0] = np.nan
        z_mm = float(np.nanmedian(roi))

        self.get_logger().info(
            f"[DATA] Depth at centroid ({u},{v}): {z_mm:.1f}mm",
            throttle_duration_sec=0.5
        )
        # =====================================================

        if np.isnan(z_mm) or z_mm <= 0:
            # =====================================================
            # BLIND SPOT FALLBACK — replace the old if/else here
            # =====================================================
            if area >= self.GRAB_AREA:
                x_cam = (u - self.cx) * 0.15 / self.fx
                if abs(x_cam) < 0.06:
                    z = 0.15
                    self.get_logger().info(
                        f"[DATA] Blind spot + centered (x={x_cam:.3f}m area={area:.0f}) → z=0.15m"
                    )
                else:
                    z = 0.15
                    self.get_logger().info(
                        f"[DATA] Blind spot but off-center x={x_cam:.3f}m — still publishing"
                    )
            # =====================================================
            else:
                self.get_logger().warn(
                    f"[DATA] Depth invalid, area={area:.0f} < GRAB_AREA={self.GRAB_AREA} "
                    f"— skipping (no fallback)",
                    throttle_duration_sec=1.0
                )
                self._publish_debug_img(cv_img, c, u, v, 0.0, area,
                                        nan_pct, hsv_pct, circularity, w_b, h_b,
                                        valid=False)
                return
        else:
            z = z_mm / 1000.0

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy

        self.get_logger().info(
            f"[DATA] Publishing: x={x_cam:.3f}m y={y_cam:.3f}m z={z:.3f}m"
        )

        self._publish_debug_img(cv_img, c, u, v, z, area,
                                nan_pct, hsv_pct, circularity, w_b, h_b,
                                valid=True)

        pt = Point()
        pt.x = x_cam
        pt.y = y_cam
        pt.z = z
        self.pub.publish(pt)

    # =====================================================
    # DEBUG IMAGE PUBLISHER
    # Overlays all key data collection values on the image
    # so you can see them in rqt_image_view while testing
    # =====================================================
    def _publish_debug_img(self, cv_img, contour, u, v, z, area,
                           nan_pct, hsv_pct, circularity, w_b, h_b, valid=True):
        debug_img = cv_img.copy()

        # Contour and centroid
        color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.drawContours(debug_img, [contour], -1, color, 2)
        x_b, y_b, _, _ = cv2.boundingRect(contour)
        cv2.rectangle(debug_img, (x_b, y_b), (x_b + w_b, y_b + h_b), (0, 255, 255), 2)
        cv2.circle(debug_img, (u, v), 5, (0, 0, 255), -1)

        # Overlay text — all key data values
        lines = [
            f"z={z:.3f}m  area={area:.0f}px",
            f"nan={nan_pct:.0f}%  hsv={hsv_pct:.1f}%",
            f"circ={circularity:.2f}  bbox={w_b}x{h_b}",
            f"centroid=({u},{v})",
        ]
        for i, line in enumerate(lines):
            cv2.putText(debug_img, line, (10, 30 + i * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

        if not valid:
            cv2.putText(debug_img, "SKIPPED", (10, 150),
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
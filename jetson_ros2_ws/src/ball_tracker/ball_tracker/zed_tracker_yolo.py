#!/usr/bin/env python3
"""
zed_tracker_yolo.py

Single node for ZED camera YOLO-based tracking (ball + bottle + net strap).

YOLO model classes:
  0 = tennis ball   → /tennis_ball_position  +  /tennis_ball_position_close
  1 = water bottle  → /bottle_position  +  TF neon_bottle
  2 = net strap     → /court_pole_position

All debug images published as CompressedImage for low bandwidth.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Point, TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import cv2
import numpy as np
import os

# Ultralytics YOLO
import torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class ZEDTrackerYolo(Node):

    def __init__(self):
        super().__init__('zed_tracker_yolo')

        # =====================================================
        # MODEL
        # =====================================================
        model_path = os.path.join(
            get_package_share_directory('ball_tracker'), 'zed_best_train_model.pt')

        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        device = 'cpu'
        self.get_logger().info(f"Using device: {device}")
        self.model = YOLO(model_path)
        self.model.to(device)

        self.CLASS_BALL   = 0
        self.CLASS_BOTTLE = 1
        self.CLASS_STRAP  = 2   # net centre strap → replaces court pole

        # =====================================================
        # TUNING
        # =====================================================
        self.CONF_THRESHOLD = 0.1    # minimum detection confidence
        self.DEPTH_ROI      = 5       # px radius for median depth sample
        self.GRAB_AREA_PX   = 175000  # px² — fallback z when depth is NaN (very close)

        # =====================================================
        # CAMERA INTRINSICS
        # =====================================================
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_img  = None
        self.bridge     = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # =====================================================
        # SUBSCRIBERS
        # =====================================================
        self.create_subscription(CameraInfo,
            '/zed/zed_node/rgb/color/rect/camera_info', self.cam_info_cb, 10)
        self.create_subscription(Image,
            '/zed/zed_node/rgb/color/rect/image',       self.rgb_cb,      10)
        self.create_subscription(Image,
            '/zed/zed_node/depth/depth_registered',     self.depth_cb,    10)

        # =====================================================
        # PUBLISHERS
        # =====================================================
        # Ball — two topics so ball_chaser.py keeps working unchanged
        self.pub_ball_far   = self.create_publisher(Point, '/tennis_ball_position',       10)
        self.pub_ball_close = self.create_publisher(Point, '/tennis_ball_position_close', 10)

        # Bottle
        self.pub_bottle = self.create_publisher(Point, '/bottle_position', 10)

        # Court pole / net strap
        self.pub_pole = self.create_publisher(Point, '/court_pole_position', 10)

        # Debug — compressed (low bandwidth, for streaming)
        self.pub_debug = self.create_publisher(
            CompressedImage, '/zed/yolo/debug_image/compressed', 10)
        # Debug — uncompressed (for Foxglove / rosbag recording)
        self.pub_debug_raw = self.create_publisher(
            Image, '/zed/yolo/debug_image', 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("ZED YOLO Tracker started!")
        self.get_logger().info(
            f"  class 0 (ball)   → /tennis_ball_position + /tennis_ball_position_close")
        self.get_logger().info(
            f"  class 1 (bottle) → /bottle_position + TF neon_bottle")
        self.get_logger().info(
            f"  class 2 (strap)  → /court_pole_position")
        self.get_logger().info(f"  conf threshold   = {self.CONF_THRESHOLD}")
        self.get_logger().info("=" * 60)

    # =====================================================
    # CAMERA INFO
    # =====================================================
    def cam_info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f"Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f}")

    # =====================================================
    # DEPTH
    # =====================================================
    def depth_cb(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    # =====================================================
    # RGB → YOLO INFERENCE
    # =====================================================
    def rgb_cb(self, msg: Image):
        if self.fx is None:
            self.get_logger().info(
                "Waiting for camera_info...", throttle_duration_sec=2.0)
            return
        if self.depth_img is None:
            self.get_logger().info(
                "Waiting for depth image...", throttle_duration_sec=2.0)
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        debug  = cv_img.copy()

        # Run YOLO
        results = self.model(cv_img, conf=self.CONF_THRESHOLD, verbose=False, device='cpu')

        best = {
            self.CLASS_BALL:   None,   # (conf, box)
            self.CLASS_BOTTLE: None,
            self.CLASS_STRAP:  None,
        }

        # Keep highest-confidence detection per class
        for result in results:
            for box in result.boxes:
                cls  = int(box.cls[0])
                conf = float(box.conf[0])
                if cls not in best:
                    continue
                if conf < self.CONF_THRESHOLD:   # explicit guard
                    continue
                if best[cls] is None or conf > best[cls][0]:
                    best[cls] = (conf, box)

        # ── Process each class ─────────────────────────────────────────
        self._handle_ball  (best[self.CLASS_BALL],   cv_img, debug)
        self._handle_bottle(best[self.CLASS_BOTTLE], cv_img, debug)
        self._handle_strap (best[self.CLASS_STRAP],  cv_img, debug)

        self._publish_debug(debug)

    # =====================================================
    # CLASS 0 — TENNIS BALL
    # =====================================================
    def _handle_ball(self, detection, cv_img, debug):
        color = (0, 255, 0)

        if detection is None:
            cv2.putText(debug, "ball: not seen", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
            return

        conf, box = detection
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        u = (x1 + x2) // 2
        v = (y1 + y2) // 2

        z = self._depth_at(u, v)

        # If depth NaN but bbox is huge → robot is right on top of ball
        if z is None:
            bbox_area = (x2 - x1) * (y2 - y1)
            if bbox_area >= self.GRAB_AREA_PX:
                z = 0.10
            else:
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(debug, f"ball: no depth ({conf:.2f})", (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                return

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy

        pt = Point()
        pt.x = x_cam
        pt.y = y_cam
        pt.z = z

        # Publish on BOTH topics — ball_chaser.py can use either
        self.pub_ball_far.publish(pt)
        self.pub_ball_close.publish(pt)

        self.get_logger().info(
            f"[BALL] z={z:.3f}m x={x_cam:.3f}m conf={conf:.2f}",
            throttle_duration_sec=0.5)

        cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
        cv2.circle(debug, (u, v), 5, (0, 0, 255), -1)
        cv2.putText(debug, f"ball z={z:.2f}m ({conf:.2f})", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    # =====================================================
    # CLASS 1 — WATER BOTTLE
    # =====================================================
    def _handle_bottle(self, detection, cv_img, debug):
        color = (255, 0, 255)

        if detection is None:
            cv2.putText(debug, "bottle: not seen", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
            return

        conf, box = detection
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        u = (x1 + x2) // 2
        v = (y1 + y2) // 2

        z = self._depth_at(u, v)
        if z is None:
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(debug, f"bottle: no depth ({conf:.2f})", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            return

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy

        pt = Point()
        pt.x = x_cam
        pt.y = y_cam
        pt.z = z
        self.pub_bottle.publish(pt)

        # Broadcast TF so ball_chaser.py can still look up neon_bottle
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'zed_camera_center'
        t.child_frame_id  = 'neon_bottle'
        t.transform.translation.x = float(x_cam)
        t.transform.translation.y = float(y_cam)
        t.transform.translation.z = float(z)
        t.transform.rotation.w    = 1.0
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"[BOTTLE] z={z:.3f}m x={x_cam:.3f}m conf={conf:.2f}",
            throttle_duration_sec=0.5)

        cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
        cv2.circle(debug, (u, v), 5, color, -1)
        cv2.putText(debug, f"bottle z={z:.2f}m ({conf:.2f})", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    # =====================================================
    # CLASS 2 — NET STRAP / COURT MARKER
    # =====================================================
    def _handle_strap(self, detection, cv_img, debug):
        color = (0, 165, 255)   # orange

        if detection is None:
            cv2.putText(debug, "strap: not seen", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 2)
            return

        conf, box = detection
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        u = (x1 + x2) // 2
        h, w = debug.shape[:2]

        # Normalised x: -1.0 (far left) → +1.0 (far right)
        x_norm = (u - w / 2.0) / (w / 2.0)

        pt = Point()
        pt.x = x_norm
        pt.y = 0.0
        pt.z = 1.0   # placeholder — only direction matters, not depth
        self.pub_pole.publish(pt)

        self.get_logger().info(
            f"[STRAP] x_norm={x_norm:.2f} conf={conf:.2f}",
            throttle_duration_sec=0.5)

        cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
        cv2.circle(debug, (u, (y1 + y2) // 2), 5, color, -1)
        cv2.putText(debug, f"strap x={x_norm:.2f} ({conf:.2f})", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    # =====================================================
    # DEPTH HELPER — ZED depth is already in metres
    # =====================================================
    def _depth_at(self, u, v):
        """Return depth in metres at pixel (u,v) using median ROI. None if invalid."""
        if self.depth_img is None:
            return None
        h, w = self.depth_img.shape[:2]
        r    = self.DEPTH_ROI
        roi  = self.depth_img[
            max(0, v - r):min(h, v + r + 1),
            max(0, u - r):min(w, u + r + 1)
        ].astype(float)
        valid = roi[np.isfinite(roi) & (roi > 0)]
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    # =====================================================
    # DEBUG IMAGE
    # =====================================================
    def _publish_debug(self, debug_img):
        stamp = self.get_clock().now().to_msg()

        # Compressed — for live streaming
        _, buf = cv2.imencode('.jpg', debug_img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        cmsg = CompressedImage()
        cmsg.header.stamp = stamp
        cmsg.format = "jpeg"
        cmsg.data   = buf.tobytes()
        self.pub_debug.publish(cmsg)

        # Uncompressed — for Foxglove / rosbag
        raw = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        raw.header.stamp = stamp
        self.pub_debug_raw.publish(raw)


def main(args=None):
    rclpy.init(args=args)
    node = ZEDTrackerYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

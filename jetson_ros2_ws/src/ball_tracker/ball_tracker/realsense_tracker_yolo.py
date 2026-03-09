#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os


class RealSenseYOLOTracker(Node):
    def __init__(self):
        super().__init__('realsense_tracker')
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        # -----------------------------------------------------
        # STATE: Start asleep (ZED handles far range)
        # -----------------------------------------------------
        self.is_active = False
        self.depth_img = None

        # Load YOLO model (Change 'yolov8n.pt' to 'yolov8n.engine' if you exported to TensorRT)
        self.get_logger().info("Loading YOLOv8n model for RealSense...")
        os.environ['YOLO_VERBOSE'] = 'False'
        self.model = YOLO('yolov8n.pt')
        self.model.to('cpu')

        # Bounding Box Thresholds
        self.STOP_WIDTH = 180  # Pixels wide when we must STOP and grab

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.cam_info_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image,
                                 '/camera/camera/aligned_depth_to_color/image_raw',
                                 self.depth_cb, 10)

        # NEW: The Compute Toggle Subscriber
        self.create_subscription(Bool, '/use_realsense', self.toggle_cb, 10)

        # Publishers
        self.pub = self.create_publisher(Point, '/tennis_ball_position_close', 10)
        #self.debug_pub = self.create_publisher(CompressedImage, '/realsense/debug_image', 10)
        self.debug_pub = self.create_publisher(Image, '/realsense/debug_image', 10)

        self.get_logger().info("RealSense YOLO tracker ready (Sleeping).")

    def toggle_cb(self, msg):
        """Wakes up or puts the tracker to sleep based on ball_chaser state"""
        if self.is_active != msg.data:
            self.is_active = msg.data
            state_str = "AWAKE" if self.is_active else "SLEEPING"
            self.get_logger().info(f"RealSense Tracker is now {state_str}")

    def cam_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]

    def depth_cb(self, msg):
        if self.is_active:
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_cb(self, msg):
        # COMPUTE SAVER: If ball is far away, do absolutely nothing.
        if not self.is_active or None in [self.fx, self.fy, self.cx, self.cy]:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLO inference (classes=[32] is 'sports ball')
        results = self.model.predict(
            source=cv_img,
            imgsz=320,
            classes=[32],
            conf=0.3,
            verbose=False,
            device='cpu'
        )

        if len(results[0].boxes) == 0:
            # =====================================================
            # YOLO FALLBACK — closest depth in center region
            # When YOLO can't detect the ball at close range
            # (fills too much of frame), use depth to find it.
            # At this point we know the ball is in front of us
            # so the closest thing in the center IS the ball.
            # =====================================================
            if self.depth_img is not None:
                h, w = self.depth_img.shape[:2]
                cx, cy = w // 2, h // 2
                margin = 100  # 200x200 center region
                center = self.depth_img[
                    cy - margin:cy + margin,
                    cx - margin:cx + margin
                ].astype(float)
                valid = center[(center > 0) & np.isfinite(center)]

                if len(valid) > 0:
                    z_mm = float(np.min(valid))  # closest point in center
                    if z_mm < 800:               # only trust if within 0.8m
                        z = z_mm / 1000.0
                        self.get_logger().info(
                            f"[FALLBACK] YOLO missed — depth fallback z={z:.3f}m",
                            throttle_duration_sec=0.5
                        )
                        pt = Point()
                        pt.x = 0.0  # assume centered — ball should be right ahead
                        pt.y = 0.0
                        pt.z = z
                        self.pub.publish(pt)
                        self._publish_debug_img(cv_img, None, valid=False,
                                                fallback_z=z)
                        return

            self._publish_debug_img(cv_img, None, valid=False)
            return
            # =====================================================

        # Get highest confidence ball
        best_box = results[0].boxes[0]
        x1, y1, x2, y2 = map(int, best_box.xyxy[0].cpu().numpy())
        conf = float(best_box.conf[0].cpu().numpy())

        # Geometry
        bbox_width = x2 - x1
        u = x1 + (bbox_width / 2.0)
        v = y1 + ((y2 - y1) / 2.0)

        # Width-to-Depth Logic
        if bbox_width >= self.STOP_WIDTH:
            z = 0.15  # Force grab distance — ball is right there
            self.get_logger().info(
                f"[YOLO] bbox_width={bbox_width}px >= STOP_WIDTH={self.STOP_WIDTH} "
                f"— forcing z=0.15m"
            )
        else:
            # Z = (focal_length_x * real_diameter_meters) / pixel_width
            z = (self.fx * 0.067) / bbox_width
            self.get_logger().info(
                f"[YOLO] bbox_width={bbox_width}px conf={conf:.2f} z={z:.3f}m",
                throttle_duration_sec=0.5
            )

        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy

        # Publish target
        pt = Point()
        pt.x, pt.y, pt.z = float(x_cam), float(y_cam), float(z)
        self.pub.publish(pt)

        # Publish debug image
        box_data = (x1, y1, x2, y2, conf, bbox_width, z)
        self._publish_debug_img(cv_img, box_data, valid=True)

    def _publish_debug_img(self, cv_img, box_data, valid=True, fallback_z=None):
        debug_img = cv_img.copy()

        if valid and box_data:
            x1, y1, x2, y2, conf, width, z = box_data

            # Draw Bounding Box
            color = (0, 255, 0)  # Green
            if width >= self.STOP_WIDTH:
                color = (0, 0, 255)  # Red if in grab zone

            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 3)

            # Draw Center Dot
            cv2.circle(debug_img, (int(x1 + width / 2), int(y1 + (y2 - y1) / 2)), 5, (255, 0, 0), -1)

            # Draw Text
            label = f"Ball {conf:.2f} | W:{width}px | Z:{z:.2f}m"
            cv2.putText(debug_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        elif fallback_z is not None:
            # Show fallback mode on screen
            cv2.putText(debug_img, f"FALLBACK depth z={fallback_z:.3f}m",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
            cv2.putText(debug_img, "YOLO miss — using center depth",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        else:
            cv2.putText(debug_img, "Searching...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)

        img_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_pub.publish(img_msg)
        # # Compress and Publish
        # compressed_msg = CompressedImage()
        # compressed_msg.header.stamp = self.get_clock().now().to_msg()
        # compressed_msg.format = "jpeg"
        # _, buf = cv2.imencode('.jpg', debug_img, [cv2.IMWRITE_JPEG_QUALITY, 50])
        # compressed_msg.data = buf.tobytes()
        # self.debug_pub.publish(compressed_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RealSenseYOLOTracker())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
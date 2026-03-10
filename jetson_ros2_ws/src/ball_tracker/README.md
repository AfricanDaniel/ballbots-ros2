# ball_tracker

ROS2 package for tennis ball, bottle, and court pole detection on the Jetson Orin Nano. Provides 3D position estimates using ZED 2i (far range) and Intel RealSense D435i (close range) cameras.

## Nodes

### Primary Nodes (currently used in production)

#### `tennis_ball_tracker`
ZED AI-based tennis ball detector using ZED SDK's built-in object detection.

- **Subscribes:**
  - `/zed/zed_node/rgb/color/rect/image` — ZED RGB image
  - `/zed/zed_node/obj_det/objects` (`zed_interfaces/ObjectsStamped`) — ZED AI detections
- **Publishes:**
  - `/tennis_ball_position` (`geometry_msgs/Point`) — 3D ball position in meters (x=lateral, y=vertical, z=forward)
  - `/tennis_ball_debug_image` — Annotated image for debugging
  - `/tennis_ball_marker` (`visualization_msgs/Marker`) — RViz visualization
- **TF:** Broadcasts `tennis_ball` frame relative to `zed_camera_center`
- **Notes:** Falls back to bounding-box focal-length estimation if ZED depth is NaN

---

#### `realsense_tracker_yolo`
Unified YOLO-based tracker for the RealSense D435i. Detects tennis ball, drop-zone bottle, and court pole in one node.

- **Subscribes:**
  - `/camera/camera/color/image_raw` — RealSense RGB
  - `/camera/camera/aligned_depth_to_color/image_raw` — Depth aligned to color
  - `/camera/camera/color/camera_info` — Camera intrinsics
- **Publishes:**
  - `/tennis_ball_position` (`geometry_msgs/Point`) — Ball position (close range)
  - `/tennis_ball_position_close` (`geometry_msgs/Point`) — Duplicate for close-range handoff signal
  - `/bottle_position` (`geometry_msgs/Point`) — Water bottle (drop-zone marker) position
  - `/court_pole_position` (`geometry_msgs/Point`) — Court net strap x-position for alignment
  - `/realsense_debug_image/compressed` — Compressed annotated debug image
- **TF:** Broadcasts `neon_bottle` frame for drop-zone navigation
- **Model:** `best_train_model.pt` (YOLOv8, custom-trained)
  - Class 0: Tennis ball
  - Class 1: Water bottle
  - Class 2: Court net strap / pole
- **Blind-spot fallback:** If depth returns NaN but bounding box area > 175,000 px², assumes z=0.10m (robot is directly on ball)

---

#### `bottle_tracker`
HSV-based detector for the drop-zone bottle and court pole using the ZED 2i.

- **Subscribes:**
  - `/zed/zed_node/rgb/color/rect/image` — ZED RGB image
  - `/zed/zed_node/depth/depth_registered` — ZED depth
  - `/zed/zed_node/rgb/color/rect/camera_info` — Camera intrinsics
- **Publishes:**
  - `/bottle_position` (`geometry_msgs/Point`) — Yellow bottle position
  - `/court_pole_position` (`geometry_msgs/Point`) — Red court pole x-position
- **TF:** Broadcasts `neon_bottle` frame
- **Detection:**
  - Bottle: Yellow HSV range, largest contour
  - Pole: Red HSV range (H: 0–10 or 165–179), requires 3 consecutive confirmations

---

### Alternative / Development Nodes

| Node | Camera | Method | Status |
|------|--------|--------|--------|
| `tennis_ball_tracker_yolo` | ZED 2i | YOLOv8 (nano, class 32) | Alternative to ZED AI |
| `tennis_ball_tracker_hsv_base` | ZED 2i | HSV filtering | Lightweight fallback |
| `tennis_ball_tracker_depth` | ZED 2i | HSV + ground plane masking | Experimental |
| `realsense_tracker_WORKING_BASE` | RealSense | HSV filtering | HSV baseline/logging |
| `bottle_tracker_yolo` | ZED 2i | YOLOv8 | Alternative bottle detect |
| `hsv_tuner` | Any | Interactive sliders | Development tool |

---

## Launch Files

### `tennis_ball_tracker.launch.xml`
Launches the full perception stack:
```
ZED 2i camera          → zed_camera.launch.py (model: zed2i)
RealSense D435i        → rs_launch.py (with aligned depth enabled)
realsense_tracker_yolo → unified ball/bottle/pole detection
bottle_tracker         → ZED-based bottle + pole detection
```

```bash
ros2 launch ball_tracker tennis_ball_tracker.launch.xml
```

---

## HSV Tuner

Use `hsv_tuner` to calibrate color ranges for your lighting conditions:

```bash
ros2 run ball_tracker hsv_tuner
```

Adjust sliders live and note the final HSV bounds to update in the tracker configs.

---

## YOLO Model

The custom model `best_train_model.pt` is located in the `ball_tracker/` Python package directory.

- **Architecture:** YOLOv8 nano (optimized for Jetson)
- **Training tool:** See `AutoLabel/` at the project root for labeling workflow

**Inference settings:**
- Confidence threshold: 0.25
- Input: 640×480 RGB
- Depth: sampled from a 5×5 pixel window at detection center

---

## Topic Summary

| Topic | Type | Description |
|-------|------|-------------|
| `/tennis_ball_position` | `geometry_msgs/Point` | Ball 3D position (ZED, far range) |
| `/tennis_ball_position_close` | `geometry_msgs/Point` | Ball 3D position (RealSense, close range) |
| `/bottle_position` | `geometry_msgs/Point` | Bottle 3D position |
| `/court_pole_position` | `geometry_msgs/Point` | Pole x-position (normalized) |
| `/tennis_ball_debug_image` | `sensor_msgs/Image` | ZED annotated debug image |
| `/realsense_debug_image/compressed` | `sensor_msgs/CompressedImage` | RealSense annotated debug image |
| `/tennis_ball_marker` | `visualization_msgs/Marker` | RViz ball marker |

## TF Frames

| Frame | Published by | Description |
|-------|-------------|-------------|
| `tennis_ball` | `tennis_ball_tracker` | Ball position in ZED camera frame |
| `neon_bottle` | `realsense_tracker_yolo`, `bottle_tracker` | Bottle position for return navigation |

---

## Dependencies

- ROS2 Humble
- `zed-ros2-wrapper` (ZED SDK ≥ 4.x)
- `realsense2_camera`
- `ultralytics` (YOLOv8)
- `opencv-python`
- `numpy`
- `zed_interfaces`
- `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`

## Build

```bash
cd jetson_ros2_ws
colcon build --packages-select ball_tracker
source install/setup.bash
```

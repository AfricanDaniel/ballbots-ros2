# ball_tracker

ROS2 package for tennis ball, bottle, and court pole detection on the Jetson Orin Nano. Provides 3D position estimates using ZED 2i (far range) and Intel RealSense D435i (close range) cameras.

## Nodes

### Primary Nodes (currently used in production)

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
- **Model:** `realsense_best_train_model.pt` (YOLOv8, custom-trained)
  - Class 0: Tennis ball
  - Class 1: Water bottle
  - Class 2: Court net strap / pole
- **Blind-spot fallback:** If depth returns NaN but bounding box area > 175,000 px², assumes z=0.10m (robot is directly on ball)

---

#### `zed_tracker_yolo`
YOLO-based tracker for the ZED 2i. Mirrors `realsense_tracker_yolo` but uses the ZED camera. Used when `use_zed:=true` alongside the RealSense tracker.

- **Subscribes:**
  - `/zed/zed_node/rgb/color/rect/camera_info` — ZED camera intrinsics
  - `/zed/zed_node/rgb/color/rect/image` — ZED RGB image
  - `/zed/zed_node/depth/depth_registered` — ZED depth (already in metres)
- **Publishes:**
  - `/tennis_ball_position` (`geometry_msgs/Point`) — Ball position (far range)
  - `/tennis_ball_position_close` (`geometry_msgs/Point`) — Same detection, duplicate for handoff signal
  - `/bottle_position` (`geometry_msgs/Point`) — Water bottle position
  - `/court_pole_position` (`geometry_msgs/Point`) — Net strap x-position (normalised)
  - `/zed/yolo/debug_image/compressed` — Compressed annotated debug image
  - `/zed/yolo/debug_image` — Raw annotated debug image (for Foxglove / rosbag)
- **TF:** Broadcasts `neon_bottle` frame relative to `zed_camera_center`
- **Model:** `zed_best_train_model.pt` (YOLOv8, custom-trained)
  - Class 0: Tennis ball
  - Class 1: Water bottle
  - Class 2: Net strap
- **Blind-spot fallback:** If depth NaN but bbox area ≥ 175,000 px², assumes z=0.10m

---

#### `zed_tracker`
HSV-based tracker for the ZED 2i. Fallback alternative to `zed_tracker_yolo`. Used when `use_zed:=true use_yolo:=false`.

- **Subscribes:**
  - `/zed/zed_node/rgb/color/rect/camera_info` — ZED camera intrinsics
  - `/zed/zed_node/rgb/color/rect/image` — ZED RGB image
  - `/zed/zed_node/depth/depth_registered` — ZED depth (already in metres)
- **Publishes:**
  - `/tennis_ball_position_close` (`geometry_msgs/Point`) — Ball position (close range via ZED)
  - `/bottle_position` (`geometry_msgs/Point`) — Blue bottle position
  - `/court_pole_position` (`geometry_msgs/Point`) — Red pole x-position (normalised)
  - `/zed/debug_image/compressed` — Ball debug image
  - `/zed/bottle_debug/compressed` — Bottle debug image
  - `/zed/pole_debug/compressed` — Pole debug image
- **TF:** Broadcasts `neon_bottle` frame relative to `zed_camera_center`
- **Detection:**
  - Ball: Yellow-green HSV range (H: 22–48)
  - Bottle: Blue HSV range (H: 100–118, S: 140+), 3-frame confirmation
  - Pole: Red HSV range (H: 0–10 or 165–179), aspect ratio ≥ 1.5, 3-frame confirmation

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

Always launches both cameras (RealSense for depth, ZED for odometry) then brings up the tracker nodes according to `use_yolo` and `use_zed`.

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_yolo` | `true` | `true` = YOLO trackers, `false` = HSV trackers |
| `use_zed` | `false` | `true` = also launch ZED tracker alongside RealSense |
| `use_rviz` | `false` | Launch RViz visualization |
| `debug_logs` | `false` | Enable verbose detection logs |

**Node combinations launched per argument:**

| `use_yolo` | `use_zed` | Nodes launched |
|-----------|----------|----------------|
| `true` (default) | `false` (default) | `realsense_tracker_yolo` |
| `true` | `true` | `realsense_tracker_yolo` + `zed_tracker_yolo` |
| `false` | `false` | `realsense_tracker` |
| `false` | `true` | `realsense_tracker` + `zed_tracker` |

```bash
# Default — RealSense YOLO only
ros2 launch ball_tracker tennis_ball_tracker.launch.xml

# RealSense YOLO + ZED YOLO (must also pass use_zed:=true to start_tennis_bot.launch.xml)
ros2 launch ball_tracker tennis_ball_tracker.launch.xml use_zed:=true

# HSV fallback — RealSense only
ros2 launch ball_tracker tennis_ball_tracker.launch.xml use_yolo:=false

# HSV fallback — RealSense + ZED
ros2 launch ball_tracker tennis_ball_tracker.launch.xml use_yolo:=false use_zed:=true
```

> **Important:** `use_zed` must be set consistently on **both** sides — Jetson (this launch) and robot (`start_tennis_bot.launch.xml`). If they differ, `ball_chaser` will subscribe to ZED topics that nothing is publishing, or vice versa.

---

## HSV Tuner

Use `hsv_tuner` to calibrate color ranges for your lighting conditions:

```bash
ros2 run ball_tracker hsv_tuner
```

Adjust sliders live and note the final HSV bounds to update in the tracker configs.

---

## YOLO Models

Two custom models in the `ball_tracker/` Python package directory:

| Model file | Used by | Camera |
|-----------|---------|--------|
| `realsense_best_train_model.pt` | `realsense_tracker_yolo` | RealSense D435i |
| `zed_best_train_model.pt` | `zed_tracker_yolo` | ZED 2i |

Both models:
- **Architecture:** YOLOv8 nano (optimized for Jetson)
- **Classes:** 0 = tennis ball, 1 = water bottle, 2 = net strap/pole
- **Training tool:** See `AutoLabel/` at the project root for labeling workflow

**Inference settings:**
- `realsense_tracker_yolo`: confidence threshold 0.25, depth sampled from 5×5 px window
- `zed_tracker_yolo`: confidence threshold 0.10, depth sampled from 5×5 px window (ZED depth already in metres)

---

## Topic Summary

| Topic | Type | Published by | Description |
|-------|------|-------------|-------------|
| `/tennis_ball_position` | `geometry_msgs/Point` | `realsense_tracker_yolo`, `zed_tracker_yolo` | Ball 3D position (far range) |
| `/tennis_ball_position_close` | `geometry_msgs/Point` | `realsense_tracker_yolo`, `realsense_tracker`, `zed_tracker_yolo`, `zed_tracker` | Ball 3D position (close range / handoff signal) |
| `/bottle_position` | `geometry_msgs/Point` | `realsense_tracker_yolo`, `zed_tracker_yolo`, `zed_tracker`, `bottle_tracker` | Bottle 3D position |
| `/court_pole_position` | `geometry_msgs/Point` | `realsense_tracker_yolo`, `zed_tracker_yolo`, `zed_tracker`, `bottle_tracker` | Pole x-position (normalised) |
| `/realsense_debug_image/compressed` | `sensor_msgs/CompressedImage` | `realsense_tracker_yolo` | RealSense annotated debug image |
| `/zed/yolo/debug_image/compressed` | `sensor_msgs/CompressedImage` | `zed_tracker_yolo` | ZED YOLO annotated debug image |
| `/zed/yolo/debug_image` | `sensor_msgs/Image` | `zed_tracker_yolo` | ZED YOLO debug image (uncompressed, for Foxglove) |
| `/zed/debug_image/compressed` | `sensor_msgs/CompressedImage` | `zed_tracker` | ZED HSV ball debug image |
| `/zed/bottle_debug/compressed` | `sensor_msgs/CompressedImage` | `zed_tracker` | ZED HSV bottle debug image |
| `/zed/pole_debug/compressed` | `sensor_msgs/CompressedImage` | `zed_tracker` | ZED HSV pole debug image |

## TF Frames

| Frame | Published by | Description |
|-------|-------------|-------------|
| `neon_bottle` | `realsense_tracker_yolo`, `zed_tracker_yolo`, `zed_tracker`, `bottle_tracker` | Bottle position for return navigation |

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

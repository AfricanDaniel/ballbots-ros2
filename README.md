# Tennis Ball Retrieval Robot

An autonomous mecanum-wheel robot that detects, chases, and retrieves tennis balls on a court using dual cameras and a servo-controlled claw.

## System Architecture

The system is split across two computers communicating over ROS2:

```
┌─────────────────────────────────────────────────────────┐
│              Jetson Orin Nano (Perception)               │
│                                                          │
│  ┌──────────────┐    ┌──────────────────────────────┐   │
│  │  ZED 2i      │    │  Intel RealSense D435i        │   │
│  │  (far range) │    │  (close range)                │   │
│  └──────┬───────┘    └──────────┬───────────────────┘   │
│         │                       │                        │
│  ┌──────▼───────────────────────▼───────────────────┐   │
│  │              ball_tracker package                 │   │
│  │  tennis_ball_tracker.py  (ZED AI detection)       │   │
│  │  realsense_tracker_yolo.py (unified YOLO tracker) │   │
│  │  bottle_tracker.py       (bottle + pole detect)   │   │
│  └──────────────────────────┬────────────────────────┘   │
└─────────────────────────────┼───────────────────────────┘
                              │  ROS2 Network
                              │  /tennis_ball_position
                              │  /tennis_ball_position_close
                              │  /bottle_position
                              │  /court_pole_position
┌─────────────────────────────▼───────────────────────────┐
│             Raspberry Pi (Robot Control)                 │
│                                                          │
│  ┌────────────────────────────────────────────────┐     │
│  │              robot_patrol package               │     │
│  │  ball_chaser.py  (full mission state machine)   │     │
│  │  minimec_gamepad.py  (manual teleop)            │     │
│  └──────────────────┬─────────────────────────────┘     │
│                     │ /cmd_vel, /arm_command,             │
│                     │ /claw_command                       │
│  ┌──────────────────▼─────────────────────────────┐     │
│  │           ballbots-ros2 packages                │     │
│  │  minimec_driver  (ODrive CAN interface)         │     │
│  │  minimec_bringup (hardware launch files)        │     │
│  └──────────────────┬─────────────────────────────┘     │
│                     │                                     │
│           ┌─────────┴──────────┐                         │
│     4x ODrive ESCs        Servo Controller               │
│     (mecanum wheels)      (arm + claw)                   │
└─────────────────────────────────────────────────────────┘
```

## Repository Structure

```
winter_project/
├── ballbots_ros2_ws/          # Robot-side workspace (runs on Raspberry Pi)
│   └── src/
│       ├── ballbots-ros2/     # Hardware interface packages
│       │   ├── minimec_driver/    # ODrive CAN motor driver
│       │   ├── minimec_bringup/   # Hardware launch files
│       │   ├── minimec_control/   # ROS2 control config
│       │   ├── minimec_description/ # URDF robot model
│       │   ├── minimec_msgs/      # Custom message types
│       │   ├── minimeclib/        # Kinematics library
│       │   └── odrive_can/        # ODrive ROS2 interface
│       └── robot_patrol/      # Autonomy and behavior package
│
├── jetson_ros2_ws/            # Perception workspace (runs on Jetson Orin Nano)
│   └── src/
│       ├── ball_tracker/      # Vision detection package
│       └── zed-ros2-wrapper/  # ZED SDK ROS2 integration
│
└── AutoLabel/                 # YOLO training data labeling tools
```

## Hardware

| Component | Details |
|-----------|---------|
| Robot chassis | Mecanum wheel platform |
| Drive motors | 4x brushless motors via ODrive ESCs (CAN bus) |
| Robot computer | Raspberry Pi |
| Perception computer | Jetson Orin Nano |
| Far-range camera | ZED 2i stereo camera |
| Close-range camera | Intel RealSense D435i RGBD camera |
| Claw actuator | 2x Dynamixel servos (arm ID=0, claw ID=1) via `/dev/ttyUSB0` |

## ROS2 Topics (Cross-System)

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/tennis_ball_position` | `geometry_msgs/Point` | Jetson → Pi | ZED far-range ball detection (x, y, z in meters) |
| `/tennis_ball_position_close` | `geometry_msgs/Point` | Jetson → Pi | RealSense close-range ball detection |
| `/bottle_position` | `geometry_msgs/Point` | Jetson → Pi | Drop-zone bottle location |
| `/court_pole_position` | `geometry_msgs/Point` | Jetson → Pi | Court net pole/strap for alignment |
| `/cmd_vel` | `geometry_msgs/Twist` | Pi internal | Velocity command to motor driver |
| `/arm_command` | `std_msgs/Float32` | Pi internal | Arm servo angle (degrees) |
| `/claw_command` | `std_msgs/Float32` | Pi internal | Claw servo angle (degrees) |

## Mission State Machine

`ball_chaser.py` implements the full autonomous mission:

```
IDLE
  └─► INIT_SCAN_BOTTLE       (lock drop-zone bottle TF)
        └─► INIT_TURN_TO_COURT   (rotate to face court / find pole)
              └─► WAITING_FOR_BALL   (wait for ball still 3s in <0.1m×0.4m window)
                    └─► CHASING_ZED       (approach with ZED, far range)
                          └─► CHASING_RS      (close approach with RealSense)
                                └─► GRABBING        (arm down → claw close → arm up)
                                      └─► TURNING_TO_BOTTLE
                                            └─► SEARCHING_BOTTLE
                                                  └─► CHASING_BOTTLE
                                                        └─► DROPPING       (arm down → claw open → arm up)
                                                              └─► SEARCHING_COURT_POLE
                                                                    └─► ALIGNING_TO_COURT
                                                                          └─► WAITING_FOR_BALL (repeat)
```

## Quick Start

### Jetson (Perception)

```bash
cd jetson_ros2_ws
colcon build --packages-select ball_tracker
source install/setup.bash

# Launch cameras + tracking nodes
ros2 launch ball_tracker tennis_ball_tracker.launch.xml
```

### Raspberry Pi (Robot Control)

```bash
cd ballbots_ros2_ws
colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=ON
source install/setup.bash

# Launch hardware drivers + ball chaser
ros2 launch robot_patrol start_tennis_bot.launch.xml

# Or manual gamepad control
ros2 run robot_patrol minimec_gamepad
```

### Start Mission

```bash
# Enable autonomous chasing via service call
ros2 service call /start_chasing std_srvs/srv/SetBool "{data: true}"
```

### Gamepad Controls (manual mode)

| Input | Action |
|-------|--------|
| Left stick Y | Forward / backward |
| Right stick X | Rotate left / right |
| A / Y buttons | Arm raise / lower |
| B / X buttons | Claw open / close |
| Right bumper | E-Stop |

## Detection Pipeline

The system supports multiple detection backends:

| Node | Camera | Method | Use Case |
|------|--------|--------|---------|
| `tennis_ball_tracker` | ZED 2i | ZED AI object detection | Far-range, high accuracy |
| `tennis_ball_tracker_yolo` | ZED 2i | YOLOv8 custom model | Alternative ZED detection |
| `tennis_ball_tracker_hsv_base` | ZED 2i | HSV color filtering | Lightweight fallback |
| `realsense_tracker_yolo` | RealSense | YOLOv8 custom model | Close range, unified (ball + bottle + pole) |
| `realsense_tracker_WORKING_BASE` | RealSense | HSV color filtering | HSV baseline |
| `bottle_tracker` | ZED 2i | HSV color filtering | Drop-zone bottle + court pole |

The custom YOLO model (`best_train_model.pt`) is trained to detect:
- Class 0: Tennis ball
- Class 1: Water bottle (drop zone marker)
- Class 2: Court net strap / pole

## Building

### Robot workspace (on Raspberry Pi)
```bash
cd ballbots_ros2_ws
colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=ON
```

### Perception workspace (on Jetson)
```bash
cd jetson_ros2_ws
colcon build --packages-select ball_tracker
```

## Package READMEs

- [robot_patrol](ballbots_ros2_ws/src/robot_patrol/README.md) — Autonomous behavior and teleop
- [minimec_driver](ballbots_ros2_ws/src/ballbots-ros2/minimec_driver/README.md) — Motor control driver
- [ball_tracker](jetson_ros2_ws/src/ball_tracker/README.md) — Vision detection nodes
- [ballbots-ros2](ballbots_ros2_ws/src/ballbots-ros2/README.md) — Hardware interface packages

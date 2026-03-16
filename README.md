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
| Drive motors | 4x brushless motors via ODrive S1 ESCs (CAN bus) |
| Robot computer | Raspberry Pi |
| Perception computer | Jetson Orin Nano |
| Far-range camera | ZED 2i stereo camera |
| Close-range camera | Intel RealSense D435i RGBD camera |
| Claw actuator | 2x Dynamixel servos (arm ID=0, claw ID=1) via `/dev/ttyUSB0` — designed and validated in simulation before hardware deployment |
| Battery | 6S LiPo, 22.2 V nominal |

### Power Distribution

```
                 ┌──────────────────────┐
                 │     6S Battery        │
                 │   22.2 V Nominal      │
                 └─────────┬────────────┘
                           │
                 ┌─────────▼────────────┐
                 │   Circuit Breaker     │
                 └─────────┬────────────┘
                           │
           ┌───────────────┴────────────────┐
           │                                │
   ┌───────▼──────────┐             ┌──────▼───────────┐
   │  E-Stop Relay     │             │  DC-DC Converters│
   │ (motor power only)│             │  (always on)     │
   └───────┬──────────┘             └────────┬─────────┘
           │                                 │
  ┌────────▼────────┐        ┌───────────────┼───────────────┐
  │ Motor Power Bus │        │               │               │
  └─────┬─────┬────┘   ┌────▼────┐     ┌────▼────┐     ┌────▼────┐
        │     │        │  5V ISO │     │  19V    │     │  12V    │
     ODrive  ODrive    │ DC-DC   │     │ DC-DC   │     │ DC-DC   │
      S1s     S1s      └────┬────┘     └────┬────┘     └────┬────┘
   (FL, RL) (FR, RR)        │               │               │
                       Raspberry Pi    Jetson Orin      Dynamixel
                          + LEDs          Nano            PHB
```

> **E-Stop** cuts motor power only — the Raspberry Pi, Jetson, and servos remain powered through the always-on DC-DC converters.

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
  │
  ▼
INIT_SCAN_BOTTLE        lock drop-zone bottle TF
  │
  ▼
INIT_TURN_TO_COURT      rotate to face court / find pole
  │
  ▼
WAITING_FOR_BALL        wait for ball still (4 frames, soft-reset)
  │
  ▼
CHASING_ZED             approach with ZED, far range  [use_zed=true only]
  │
  ▼
CHASING_RS              close approach with RealSense
  │
  ▼
GRABBING                arm down → claw close → arm up
  │
  ▼
TURNING_TO_BOTTLE       rotate to known bottle direction
  │
  ▼
SEARCHING_BOTTLE        sweep scan until bottle visible
  │
  ▼
CHASING_BOTTLE          drive to bottle, stop at 0.6 m
  │
  ▼
DROPPING                arm down → claw open → arm up
  │
  ▼
SEARCHING_COURT_POLE    rotate left until pole visible
  │
  ▼
ALIGNING_TO_COURT       nudge right to face court
  │
  ▼
WAITING_FOR_BALL        (repeat)
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

### HSV vs YOLO Trade-offs

**Tennis Ball**

| | HSV | YOLOv8 |
|-|-----|--------|
| **Pro** | Extremely fast, no training data needed | Reliable at distance under varied/harsh lighting |
| **Pro** | Tuning a color range takes minutes | Lighting changes and shadows don't affect quality |
| **Con** | Highly lighting-dependent — shadows or sun can push values out of threshold | Requires manually annotated court footage |
| **Con** | False triggers on any neon-yellow object on court | Slower inference; robot must reduce speed to stay in sync |

**Water Bottle (Drop Zone)**

| | HSV | YOLOv8 |
|-|-----|--------|
| **Pro** | Any distinctively colored object works; zero latency | No risk of confusing court markings or cones for home |
| **Pro** | Swap colors in config to change the landmark | Works across lighting conditions without re-tuning |
| **Con** | Color must not appear elsewhere in the scene | Model not trained on diverse bottle shapes — unusual bottles may fail |
| **Con** | Requires manually selecting a unique landmark color | Requires retraining if the home object changes |

**Court Net Strap (Court Orientation)**

The net strap is the same white as court lines, making HSV filtering impractical without physically attaching a colored marker to the strap before each session. The YOLO model detects the strap by shape and position within the net — no court modification required.

| | HSV | YOLOv8 |
|-|-----|--------|
| **Con** | Requires a colored marker taped to the strap before every session | Needs representative training data of the strap |
| **Con** | Court lines are also white — impossible to isolate otherwise | — |
| **Pro** | — | Works on any standard court with no physical setup |
| **Pro** | — | Consistent across different net colors and lighting |

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

## Key Technical Challenges

| Challenge | Solution |
|-----------|----------|
| **Camera blind spots** | ZED has a ~0.6 m minimum range; RealSense handles the close zone. `ball_chaser` seamlessly hands off between cameras at `tilt_start_dist = 0.6 m`. |
| **Ghost frame oscillation** | YOLO detections are noisy. Angular controllers use low gains and caps so a single phantom detection can't yank the robot sideways. A 4-second alignment timeout forces forward motion if the robot oscillates in place. |
| **Ball stillness detection** | Rolling balls are ignored. 4 consecutive frames must fall within 0.10 m laterally / 0.50 m in depth. A soft-reset (decrement by 1 on a bad frame) prevents one noisy detection from wiping accumulated stability. |
| **Drop zone exclusion** | After dropping a ball, its world-frame position is logged. `WAITING_FOR_BALL` ignores any detection within 0.2 m of a known drop location to prevent re-chasing just-released balls. |
| **ODrive initialization** | All 4 motor axes must enter closed-loop control before the driver accepts any wheel commands. Retry logic clears faults and re-attempts up to 5 times per axis. |
| **Distributed compute** | Vision (YOLO, ZED SDK) runs on the Jetson GPU; motion control runs on the Pi — coordinated purely over ROS2 topics across the network with no shared memory. |
| **E-Stop safety** | The E-Stop relay cuts motor power only. Compute (Pi + Jetson) and servos run through always-on DC-DC converters, so the robot's brain stays live for safe inspection and recovery after a stop. |

## Package READMEs

- [robot_patrol](ballbots_ros2_ws/src/robot_patrol/README.md) — Autonomous behavior and teleop
- [minimec_driver](ballbots_ros2_ws/src/ballbots-ros2/minimec_driver/README.md) — Motor control driver
- [ball_tracker](jetson_ros2_ws/src/ball_tracker/README.md) — Vision detection nodes
- [ballbots-ros2](ballbots_ros2_ws/src/ballbots-ros2/README.md) — Hardware interface packages

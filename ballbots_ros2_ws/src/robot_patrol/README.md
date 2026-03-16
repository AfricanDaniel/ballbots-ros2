# robot_patrol

ROS2 Python package for autonomous tennis ball retrieval behavior and manual gamepad control. Runs on the Raspberry Pi onboard the robot.

## Nodes

### `ball_chaser` — Full Autonomous Mission
The primary node implementing the complete ball-retrieval mission as a state machine.

- **Parameters:**
  - `use_zed` (bool, default `false`) — when `false`, `ball_chaser` uses `/tennis_ball_position_close` for all ball chasing (RealSense only); when `true`, it uses `/tennis_ball_position` for far-range (`CHASING_ZED` state) and hands off to `/tennis_ball_position_close` at close range (`CHASING_RS` state).

- **Subscribes:**
  | Topic | Type | Active when |
  |-------|------|-------------|
  | `/tennis_ball_position` | `geometry_msgs/Point` | `use_zed:=true` — ZED far-range detection |
  | `/tennis_ball_position_close` | `geometry_msgs/Point` | always — RealSense close-range detection |
  | `/bottle_position` | `geometry_msgs/Point` | always |
  | `/court_pole_position` | `geometry_msgs/Point` | always |
  | `/zed/zed_node/odom` | `nav_msgs/Odometry` | always — ZED visual odometry |

- **Publishes:**
  | Topic | Type | Description |
  |-------|------|-------------|
  | `/cmd_vel` | `geometry_msgs/Twist` | Drive commands (linear x/y, angular z) |
  | `/arm_command` | `std_msgs/Float32` | Arm servo angle in degrees |
  | `/claw_command` | `std_msgs/Float32` | Claw servo angle in degrees |

- **Services:**
  | Service | Type | Description |
  |---------|------|-------------|
  | `/start_chasing` | `std_srvs/SetBool` | Start (`true`) or stop (`false`) the mission |

- **TF lookups:** `neon_bottle` — used for return-to-drop-zone navigation

#### State Machine

```
IDLE
  │  (service call: /start_chasing true)
  ▼
INIT_SCAN_BOTTLE          Lock drop-zone bottle transform (TF neon_bottle)
  ▼
INIT_TURN_TO_COURT        Rotate 40° left to face court
  │                         (if USE_COURT_POLE: search for red pole instead)
  ▼
WAITING_FOR_BALL          Wait until ball is stationary for 4 consecutive frames
  │                         (dx < 0.10m, dz < 0.50m; noisy frames decrement count
  │                          by 1 instead of resetting — soft reset)
  ▼
CHASING_ZED               Drive toward ball using ZED far-range detection
  │                         (proportional angular + linear speed)
  │                         (handoff to RS when z < tilt_start_dist = 0.6m)
  ▼
CHASING_RS                Two-phase close approach with RealSense:
  │   Phase 1 — Align: stop and rotate until |err_x| < 0.10m
  │             (forced forward after 4s oscillation timeout)
  │   Phase 2 — Drive:
  │             z > tilt_start_dist (0.6m): fast approach up to 0.4 m/s
  │             z ≤ tilt_start_dist (0.6m): slow approach max 0.15 m/s
  │             (stops and grabs when z ≤ grab_dist = 0.30m)
  ▼
GRABBING                  Arm sweep down → pause → claw close → arm raise
  ▼
TURNING_TO_BOTTLE         Rotate to face locked bottle direction (from odometry)
  ▼
SEARCHING_BOTTLE          Scan sweep until bottle visible
  │                         (0.6 rad/s spin, 0.5s spin / 0.3s pause cycles)
  ▼
CHASING_BOTTLE            Drive toward bottle at up to 0.4 m/s, stop at 0.6m
  │                         (always drives forward while correcting heading;
  │                          no deceleration ramp — speed applied instantly)
  ▼
DROPPING                  Arm sweep down → pause → claw open → arm raise
  ▼
SEARCHING_COURT_POLE      Rotate left until court pole visible
  │  (if USE_COURT_POLE=False: skip to WAITING_FOR_BALL)
  ▼
ALIGNING_TO_COURT         Nudge right to center on pole
  ▼
WAITING_FOR_BALL          (repeat mission)
```

#### Key Parameters (from `robot_config.yaml`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `arm_raised_deg` | 270° | Arm fully raised |
| `arm_lowered_deg` | 177° | Arm lowered for grab/drop |
| `claw_open_deg` | 152° | Claw open |
| `claw_closed_deg` | 175° | Claw closed |
| `grab_dist` | 0.30 m | Distance to stop and grab ball |
| `tilt_start_dist` | 0.60 m | Distance to switch from fast to slow approach (also ZED→RS handoff) |
| `target_dist` | 0.10 m | Proportional speed zero-point during final approach |
| `bottle_approach_dist` | 0.6 m | Distance to stop at drop zone |
| `BALL_STILL_THRESHOLD_X` | 0.10 m | Max x-change between frames to count as still |
| `BALL_STILL_THRESHOLD_Z` | 0.50 m | Max z-change between frames to count as still (< 4 m only) |
| `BALL_STILL_FRAMES` | 4 | Consecutive still frames required before chasing |
| `ALIGN_TIMEOUT` | 4.0 s | Force forward motion after this many seconds of alignment oscillation |
| `GRAB_CONFIRM_NEEDED` | 2 | Consecutive detections at grab distance before triggering grab |

---

### `minimec_gamepad` — Manual Teleop

Xbox / PS4 gamepad control for manual operation.

- **Subscribes:** `/joy` (`sensor_msgs/Joy`)
- **Publishes:** `/cmd_vel` (`geometry_msgs/Twist`), `/arm_command`, `/claw_command`

| Control | Action |
|---------|--------|
| Left stick Y-axis | Forward / backward |
| Right stick X-axis | Rotate left / right |
| A button | Arm raise |
| Y button | Arm lower |
| B button | Claw open |
| X button | Claw close |
| Right bumper (RB) | E-Stop (zero all motion) |

```bash
ros2 run robot_patrol minimec_gamepad
```

---

### Alternative Ball Chaser Nodes

These are earlier versions kept for reference and testing:

| Node | Method | Notes |
|------|--------|-------|
| `ball_chaser_hsv_base` | HSV tracking | Simpler FSM, 5s timed forward creep before grab |
| `ball_chaser_yolo_0` | YOLO | Basic version, 21s handoff pause |
| `ball_chaser_yolo_1` | YOLO | Velocity smoother (currently disabled — early return at line 248) |

---

## Launch Files

### `start_tennis_bot.launch.xml`

Launches the minimec hardware drivers, `ball_chaser`, and optionally `servo_control`.

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_servo` | `true` | Launch the servo control node |
| `use_zed` | `false` | `true` = `ball_chaser` subscribes to ZED ball topics (far range) in addition to RealSense |

```bash
# Default — RealSense only
ros2 launch robot_patrol start_tennis_bot.launch.xml

# RealSense + ZED (must also pass use_zed:=true to tennis_ball_tracker.launch.xml on Jetson)
ros2 launch robot_patrol start_tennis_bot.launch.xml use_zed:=true

# No servo (e.g. hardware not connected)
ros2 launch robot_patrol start_tennis_bot.launch.xml use_servo:=false
```

> **Important:** `use_zed` must match what is passed to `tennis_ball_tracker.launch.xml` on the Jetson. Mismatching the two will result in `ball_chaser` waiting for topics that nothing is publishing.

---

## Configuration

`config/robot_config.yaml` contains all hardware-specific parameters:

```yaml
/**:
  ros__parameters:
    device_name: '/dev/ttyUSB0'
    baudrate: 57600
    arm_id: 0                  # Dynamixel servo ID for arm
    claw_id: 1                 # Dynamixel servo ID for claw
    arm_raised_deg: 270.0      # Arm fully raised (degrees)
    arm_lowered_deg: 177.0     # Arm lowered for grab/drop (degrees)
    claw_open_deg: 152.0       # Claw open (degrees)
    claw_closed_deg: 175.0     # Claw closed (degrees)
    target_dist: 0.1           # Speed zero-point during final approach (metres)
    grab_dist: 0.30            # Stop and grab when ball is this close (metres)
    tilt_start_dist: 0.6       # Switch fast→slow approach; ZED→RS handoff (metres)
    use_zed: false             # false = RealSense only; true = ZED far + RS close
```

---

## Camera Modes

### Mode 1 — RealSense only (default, `use_zed:=false`)

`ball_chaser` skips the `CHASING_ZED` state and goes straight to `CHASING_RS` using `/tennis_ball_position_close` from `realsense_tracker_yolo` (or `realsense_tracker`).

```bash
# Jetson
ros2 launch ball_tracker tennis_ball_tracker.launch.xml

# Robot
ros2 launch robot_patrol start_tennis_bot.launch.xml
```

### Mode 2 — ZED far + RealSense close (`use_zed:=true`)

`ball_chaser` uses `/tennis_ball_position` (from `zed_tracker_yolo` or `zed_tracker`) for the `CHASING_ZED` state, then hands off to `/tennis_ball_position_close` (RealSense) for the `CHASING_RS` state.

```
Ball z > tilt_start_dist (0.6m)  →  CHASING_ZED  (ZED far range, better 3D accuracy at distance)
Ball z < tilt_start_dist (0.6m)  →  CHASING_RS   (RealSense close range, handles ZED blind spot)
Ball not visible                 →  Blind-spot creep (small fixed forward motion)
```

The RealSense's `/tennis_ball_position_close` topic also acts as a trigger signal — when it publishes, the state machine transitions from `CHASING_ZED` to `CHASING_RS`.

```bash
# Jetson — must match use_zed:=true
ros2 launch ball_tracker tennis_ball_tracker.launch.xml use_zed:=true

# Robot
ros2 launch robot_patrol start_tennis_bot.launch.xml use_zed:=true
```

---

## Drop Zone Exclusion

After dropping a ball, its position is logged. The `WAITING_FOR_BALL` state ignores any ball detections within 0.3m of a known drop zone to prevent re-chasing just-dropped balls.

---

## Dependencies

- ROS2 Humble
- `geometry_msgs`, `nav_msgs`, `std_msgs`, `std_srvs`, `sensor_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `robot_patrol` depends on `minimec_msgs` for wheel command types

## Build

```bash
cd ballbots_ros2_ws
colcon build --packages-select robot_patrol
source install/setup.bash
```

## Run

```bash
# Autonomous mission
ros2 run robot_patrol ball_chaser

# Then start the mission
ros2 service call /start_chasing std_srvs/srv/SetBool "{data: true}"

# Manual control
ros2 run robot_patrol minimec_gamepad
```

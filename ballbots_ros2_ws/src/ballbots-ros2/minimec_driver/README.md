# minimec_driver

ROS2 C++ package that bridges between ROS2 velocity commands and the 4 ODrive ESCs controlling the mecanum wheels. Handles ODrive initialization, closed-loop startup with retry logic, and real-time wheel velocity control.

## Nodes

### `minimec_driver`

- **Subscribes:**
  | Topic | Type | Description |
  |-------|------|-------------|
  | `/wheel_cmd` | `minimec_msgs/WheelCommands` | Per-wheel velocity commands (FL, FR, RR, RL) |
  | `/odrive_axis0/controller_status` | `odrive_can/ControllerStatus` | FL motor feedback |
  | `/odrive_axis1/controller_status` | `odrive_can/ControllerStatus` | FR motor feedback |
  | `/odrive_axis2/controller_status` | `odrive_can/ControllerStatus` | RR motor feedback |
  | `/odrive_axis3/controller_status` | `odrive_can/ControllerStatus` | RL motor feedback |

- **Publishes:**
  | Topic | Type | Description |
  |-------|------|-------------|
  | `/odrive_axis0/control_message` | `odrive_can/ControlMessage` | FL motor velocity command |
  | `/odrive_axis1/control_message` | `odrive_can/ControlMessage` | FR motor velocity command |
  | `/odrive_axis2/control_message` | `odrive_can/ControlMessage` | RR motor velocity command |
  | `/odrive_axis3/control_message` | `odrive_can/ControlMessage` | RL motor velocity command |
  | `/joint_states` | `sensor_msgs/JointState` | Wheel positions for visualization |

- **Services (used internally for initialization):**
  | Service | Type | Description |
  |---------|------|-------------|
  | `/odrive_axisN/request_axis_state` | `odrive_can/SetAxisState` | Set ODrive closed-loop mode |
  | `/odrive_axisN/clear_errors` | `std_srvs/Trigger` | Clear ODrive fault states |

## Wheel Layout and Motor Directions

```
       Front
  FL(0) ──── FR(1)
    │            │
  RL(3) ──── RR(2)
       Rear
```

- FL and RL motors run in reverse direction (negated velocity command) due to physical mounting orientation.
- Gear ratio: **2.0** — wheel velocity is multiplied by this before sending to ODrive.

## Initialization Sequence

On startup, the driver:
1. Waits for all 4 ODrive nodes to be available
2. Clears any existing errors on all axes
3. Commands each axis into closed-loop velocity control mode (`AXIS_STATE_CLOSED_LOOP_CONTROL = 8`)
4. Retries up to 5 times per axis if the transition fails
5. Only begins accepting wheel commands once all 4 axes are in closed-loop mode

## Configuration

`config/params.yaml` — configures topic names and ODrive-to-wheel axis mapping:

```yaml
minimec_driver:
  ros__parameters:
    wheel_cmd_topic: /wheel_cmd
    joint_states_topic: /joint_states
    # Axis assignment: [FL, FR, RR, RL]
    odrive_axes: [0, 1, 2, 3]
    gear_ratio: 2.0
```

## Launch Files

- `launch_driver.launch.py` — Launches `minimec_driver` with the params file.

```bash
ros2 launch minimec_driver launch_driver.launch.py
```

## Dependencies

- ROS2 Humble
- `minimec_msgs` (custom `WheelCommands` message)
- `odrive_can` (ODrive ROS2 interface)
- `sensor_msgs`, `std_srvs`

## Build

```bash
cd ballbots_ros2_ws
colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=ON --packages-select minimec_driver
source install/setup.bash
```

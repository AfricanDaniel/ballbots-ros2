# ballbots-ros2

Hardware interface packages for the mecanum-wheel tennis ball robot. These packages handle motor control, robot description, and hardware bringup. They run on the Raspberry Pi.

## Packages

| Package | Type | Description |
|---------|------|-------------|
| `minimec_driver` | C++ node | ODrive CAN motor driver — translates wheel velocity commands to ODrive control messages |
| `minimec_bringup` | Launch | Hardware startup launch files — brings up drivers, teleop, and control |
| `minimec_control` | Config | ROS2 Control configuration (controllers, hardware interface) |
| `minimec_description` | URDF | Robot URDF model and meshes |
| `minimec_msgs` | Messages | Custom message types: `WheelCommands` (fl, fr, rr, rl float velocities) |
| `minimeclib` | C++ library | Mecanum wheel kinematics library |
| `odrive_can` | Vendor | ODrive ROS2 CAN interface (topics, services, messages for ODrive ESCs) |

## Build

```bash
cd ballbots_ros2_ws

# Build for robot (enables hardware-dependent code)
colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=ON

source install/setup.bash
```

## Launch

```bash
# Bring up full robot hardware + teleop
ros2 launch minimec_bringup launch_minimec.launch.py cmd_src:=teleop

# Bring up hardware only (command from external source, e.g. ball_chaser)
ros2 launch minimec_bringup launch_minimec.launch.py
```

## Useful Commands

```bash
# Manual wheel velocity test (all 4 wheels at 2.0 rad/s)
ros2 topic pub --rate 10 /wheel_cmd minimec_msgs/msg/WheelCommands "{fl: 2.0, fr: 2.0, rr: 2.0, rl: 2.0}"

# Drive forward at 1.0 m/s via cmd_vel
ros2 topic pub --rate 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Topic Flow

```
/cmd_vel (Twist)
    │
    ▼
[minimec_control / kinematics]
    │
    ▼
/wheel_cmd (WheelCommands: fl, fr, rr, rl)
    │
    ▼
[minimec_driver]
    │
    ├─► /odrive_axis0/control_message  (FL)
    ├─► /odrive_axis1/control_message  (FR)
    ├─► /odrive_axis2/control_message  (RR)
    └─► /odrive_axis3/control_message  (RL)
```

## See Also

- [minimec_driver README](minimec_driver/README.md) — detailed driver documentation
- [robot_patrol README](../../robot_patrol/README.md) — autonomy layer that publishes `/cmd_vel`

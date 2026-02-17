# ballbots-ros2


# On Robot
- ros2 topic pub --rate 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
- ros2 launch minimec_bringup launch_minimec.launch.py cmd_src:=teleop
- colcon build --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=ON
- ros2 topic pub --rate 10 /wheel_cmd minimec_msgs/msg/WheelCommands "{fl: 2.0, fr: 2.0, rr: 2.0, rl: 2.0}"
- colcon build --packages-select robot_patrol
- ros2 run robot_patrol square_move
- ros2 launch realsense2_camera rs_launch.py
- ros2 run robot_patrol ball_chaser

# On compu 
- ros2 launch minimec_bringup launch_command.launch.py cmd_src:=teleop
- colcon buil-cmake-args -DBUILD_TESTING=OFF -DBUILD_ROBOT=OFF
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_src',
            default_value='teleop',
            description="controls the source of the command velocity commands.",
            choices=["teleop", "path"]),

        # 1. HARDWARE: Start ODrives IMMEDIATELY
        IncludeLaunchDescription(
            YAMLLaunchDescriptionSource(
                [FindPackageShare("minimec_bringup"), '/launch',
                 '/launch_odrives.yaml']
            )
        ),

        # 2. ROBOT STATE PUBLISHER: Start IMMEDIATELY
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {"robot_description":
                     Command([ExecutableInPackage("xacro", "xacro"), " ",
                              PathJoinSubstitution(
                                  [FindPackageShare("minimec_description"), "urdf",
                                   "minimec.urdf.xacro"])])}
            ]
        ),

        # 3. SOFTWARE: Start Driver AFTER 5 SECONDS
        # This gives the CAN bus time to initialize and ODrives time to boot
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("minimec_driver"), '/launch',
                         '/launch_driver.launch.py']
                    ),
                    launch_arguments=[['cmd_src', LaunchConfiguration('cmd_src')]]
                ),

                # Optional: Start Control after Driver
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [FindPackageShare("minimec_control"), '/launch',
                         '/launch_control.launch.py']
                    ),
                ),
            ]
        ),
    ])
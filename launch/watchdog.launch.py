from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ignore_joint_limits_arg = DeclareLaunchArgument(
        "ignore_joint_limits",
        default_value="False",
        description="If set to true, watchdog won't enforce joint position limits",
    )

    watchdog_config_filepath = PathJoinSubstitution(
        [FindPackageShare("go2_control_interface"), "config", "default_limits.yaml"]
    )

    n_fails_arg = DeclareLaunchArgument(
        "n_fails",
        default_value="2",
        description="How many consecutive check without receiving any joint command is allowed before killing the robot.",
    )

    freq_arg = DeclareLaunchArgument("freq", default_value="100", description="How many checks per seconds to perform")

    return LaunchDescription(
        [
            ignore_joint_limits_arg,
            n_fails_arg,
            freq_arg,
            Node(
                package="go2_control_interface",
                executable="watchdog_node.py",
                name="watchdog",
                output="screen",
                parameters=[
                    watchdog_config_filepath,
                    {
                        "ignore_joint_limits": LaunchConfiguration("ignore_joint_limits"),
                        "n_fails": LaunchConfiguration("n_fails"),
                        "freq": LaunchConfiguration("freq"),
                    },
                ],
            ),
        ]
    )

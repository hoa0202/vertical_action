from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pose_topic", default_value="/liorf/mapping/odometry", description="Pose topic (e.g. /liorf/mapping/odometry, /amcl_pose)"),
        DeclareLaunchArgument("pose_type", default_value="odom", description="odom | amcl | pose_stamped"),
        Node(
            package="vertical_action",
            executable="record_line_positions_node",
            name="record_line_positions_node",
            output="screen",
            parameters=[{
                "pose_topic": LaunchConfiguration("pose_topic"),
                "pose_type": LaunchConfiguration("pose_type"),
            }],
        ),
    ])

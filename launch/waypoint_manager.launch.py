from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='neurofly1',
            description='Robot namespace'
        ),
        Node(
            package='waypoint_manager',
            executable='waypoints_gui',
            name='waypoints_gui',
            namespace=LaunchConfiguration('robot'),
            output='screen',
            parameters=[
                {'pose_topic': '/goal_pose'},
                {'output_topic': 'waypoints'}
            ]
        )
    ])

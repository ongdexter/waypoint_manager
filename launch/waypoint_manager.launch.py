from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_manager',
            executable='waypoints_gui',
            name='waypoints_gui',
            output='screen',
            parameters=[
                {'pose_topic': '/goal_pose'},
                {'point_topic': '/clicked_point'},
                {'output_topic': '/waypoints_path'},
                {'default_frame': 'map'},
            ]
        )
    ])

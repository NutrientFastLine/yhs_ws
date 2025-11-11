import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                {'target_frame': 'body'},
                {'transform_tolerance': 0.01},
                {'min_height': 0.0},
                {'max_height': 1.0},
                {'angle_min': -3.14159},
                {'angle_max': 3.14159},
                {'angle_increment': 0.0087},
                {'scan_time': 10.0},
                {'range_min': 0.05},
                {'range_max': 30.0},
                {'use_inf': True},
                {'inf_epsilon': 1.0}
            ],
            remappings=[
                ('cloud_in', '/cloud_registered')
            ]
        )
    ])
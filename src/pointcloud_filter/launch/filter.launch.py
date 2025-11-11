from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_filter',
            executable='pointcloud_filter_node',
            name='livox_pointcloud_filter',
            parameters=[{
                'robot_min': [-1.0, -0.40, -0.4],
                'robot_max': [0.40, 0.40, 0.4],
                'rear_angle_threshold': 140.0,
                'enable_angle_filter': True  # 禁用角度过滤
            }],
            output='screen'
        )
    ])

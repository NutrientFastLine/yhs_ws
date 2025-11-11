import os
from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('turn_on_yhs_robot')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    ekf_imu_config = os.path.join(pkg_dir, 'config', 'ekf_imu.yaml')
    
    # Launch 参数
    use_imu = LaunchConfiguration('use_imu', default='true')
    publish_tf = LaunchConfiguration('publish_tf', default='true')

    declare_use_imu_cmd = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Use IMU EKF configuration if true'
    )

    declare_publish_tf_cmd = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF from EKF node'
    )

    # 使用 RewrittenYaml 替换参数
    ekf_param_substitutions = {'publish_tf': publish_tf}
    ekf_rewritten = RewrittenYaml(
        source_file=ekf_config,
        root_key='',
        param_rewrites=ekf_param_substitutions,
        convert_types=True
    )

    ekf_imu_param_substitutions = {'publish_tf': publish_tf}
    ekf_imu_rewritten = RewrittenYaml(
        source_file=ekf_imu_config,
        root_key='',
        param_rewrites=ekf_imu_param_substitutions,
        convert_types=True
    )

    # Launch 描述
    return LaunchDescription([
        declare_use_imu_cmd,
        declare_publish_tf_cmd,

        Node(
            condition=IfCondition(use_imu),
            package='robot_localization',
            executable='ekf_node',
            name='imu_ekf_filter_node',
            parameters=[ekf_imu_rewritten],
            remappings=[('/odometry/filtered', 'odom_combined')],
            output='screen'
        ),

        Node(
            condition=UnlessCondition(use_imu),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_rewritten],
            remappings=[('/odometry/filtered', 'odom_combined')],
            output='screen'
        ),
    ])

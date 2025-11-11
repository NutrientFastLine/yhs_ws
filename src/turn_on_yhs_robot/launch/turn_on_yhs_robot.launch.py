import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # === 获取路径 ===
    bringup_dir = get_package_share_directory('turn_on_yhs_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    imu_config = Path(bringup_dir, 'config', 'imu.yaml')

    # === 参数声明 ===
    use_imu = LaunchConfiguration('use_imu', default='false')
    publish_tf = LaunchConfiguration('publish_tf', default='false')

    use_imu_dec = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Enable IMU and robot_ekf'
    )

    publish_tf_dec = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Whether EKF publishes TF (true/false)'
    )

    # === 子Launch文件 ===
    yhs_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'yhs_vehicles_interface.launch.py')),
        launch_arguments={'use_yhs_imu': use_imu}.items(),
    )

    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'yhs_ekf.launch.py')),
        launch_arguments={
            'use_imu': use_imu,
            'publish_tf': publish_tf
        }.items(),
    )

    yhs_mode_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )

    # === 静态TF ===
    base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=['0.33', '0', '-0.21', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    base_to_gyro = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=['0', '0', '0.28', '0', '0', '0', 'base_link', 'gyro_link'],
        condition=IfCondition(use_imu),  # 仅在 use_imu == true 时发布此TF
    )

    base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=['0.66', '0', '0.40', '0', '0', '0', 'base_link', 'livox_frame'],
    )

    # === IMU滤波节点 ===
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[imu_config],
        condition=IfCondition(use_imu),
    )

    # === 关节状态发布器 ===
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # === Launch描述 ===
    ld = LaunchDescription()

    ld.add_action(use_imu_dec)
    ld.add_action(publish_tf_dec)
    ld.add_action(yhs_mode_description)
    ld.add_action(yhs_robot)
    ld.add_action(base_to_footprint)
    ld.add_action(base_to_gyro)
    ld.add_action(base_to_lidar)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)
    ld.add_action(robot_ekf)

    return ld

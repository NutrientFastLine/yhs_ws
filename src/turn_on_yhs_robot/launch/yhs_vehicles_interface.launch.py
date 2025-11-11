import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """根据 use_yhs_imu 参数动态构建节点列表"""
    nodes = []

    use_imu = LaunchConfiguration('use_yhs_imu').perform(context)
    parameter_file = LaunchConfiguration('params_file').perform(context)

    # 获取核心包目录
    can_control_share_dir = get_package_share_directory('yhs_can_control')
    turn_on_share_dir = get_package_share_directory('turn_on_yhs_robot')

    # yhs_can_control节点
    yhs_can_control_node = Node(
        package='yhs_can_control',
        executable='yhs_can_control_node',
        name='yhs_can_control_node',
        output='screen',
        parameters=[parameter_file],
        remappings=[('/odom', '/yhs_odom')]
    )
    nodes.append(yhs_can_control_node)

    # nav2_chassis_adapter节点
    nav2_chassis_adapter_node = Node(
        package='turn_on_yhs_robot',
        executable='nav2_chassis_adapter',
        name='nav2_chassis_adapter',
        output='screen'
    )
    nodes.append(nav2_chassis_adapter_node)

    # 仅当 use_yhs_imu = true 时，才加载 IMU 启动文件
    if use_imu.lower() == 'true':
        fdilink_ahrs_share_dir = get_package_share_directory('fdilink_ahrs')
        imu_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fdilink_ahrs_share_dir, 'launch', 'ahrs_driver.launch.py')
            )
        )
        nodes.append(imu_launch)

    return nodes


def generate_launch_description():
    # 声明参数
    use_yhs_imu_declare = DeclareLaunchArgument(
        'use_yhs_imu',
        default_value='false',
        description='If true, use yhs_imu'
    )

    can_control_share_dir = get_package_share_directory('yhs_can_control')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(can_control_share_dir, 'params', 'cfg.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # 构建LaunchDescription
    ld = LaunchDescription()
    ld.add_action(use_yhs_imu_declare)
    ld.add_action(params_declare)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

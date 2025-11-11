import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('yhs_chassis_description')
    
    # URDF 文件路径
    urdf_file = os.path.join(pkg_path, 'urdf', 'yhs_fr07pro.urdf')
    
    # RVIZ 配置文件路径
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'urdf.rviz')
    
    # 读取 URDF 内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # 启动节点
    return LaunchDescription([
        # 设置机器人描述参数
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
    ])


import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 设置包路径
    fast_lio_dir = get_package_share_directory('fast_lio')
    fast_lio_localization_dir = get_package_share_directory('fast_lio_localization')
    octomap_server2_dir = get_package_share_directory('octomap_server2')

    # 声明参数
    rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='true',
        description='是否启动Rviz'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='是否使用仿真时间'
    )
    
    map_arg = DeclareLaunchArgument(
        'map', 
        default_value=os.path.join(fast_lio_dir, 'PCD', 'scans.pcd'),
        description='全局地图PCD文件路径'
    )
    
    # fast_lio节点配置
    fast_lio_config = PathJoinSubstitution([
        fast_lio_dir, 'config', 'mid360.yaml'
    ])
    
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[fast_lio_config]
    )
    
    # Rviz节点
    rviz_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', PathJoinSubstitution([
                    fast_lio_dir, 'rviz', 'sentry_build_map.rviz'
                ])],
                output='screen'
            )
        ]
    )
    
    # 包含其他launch文件
    pointcloud_to_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(fast_lio_dir, 'launch', 'Pointcloud2Map.launch.py')
        ])
    )
    
    # pointcloud_to_map_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(octomap_server2_dir, 'launch', 'octomap_server_launch.py')
    #     ])
    # )

    pointscloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(fast_lio_localization_dir, 'launch', 'PointsCloud2toLaserscan.launch.py')
        ])
    )
    
    # 静态TF发布
    tf_pub_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_1',
        arguments=['-0.30', '0', '0', '0', '0', '0', 'body', 'body_foot']
    )
    
    tf_pub_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_2',
        arguments=['-0.30', '0', '0', '0', '0', '0', 'camera_init', 'robot_foot_init']
    )
    
    # 被注释掉的节点（保持原样）
    # global_localization_node = Node(
    #     package='fast_lio_localization',
    #     executable='global_localization.py',
    #     name='global_localization',
    #     output='screen'
    # )
    
    # transform_fusion_node = Node(
    #     package='fast_lio_localization',
    #     executable='transform_fusion.py',
    #     name='transform_fusion',
    #     output='screen'
    # )
    
    # map_publisher_node = Node(
    #     package='pcl_ros',
    #     executable='pcd_to_pointcloud',
    #     name='map_publishe',
    #     output='screen',
    #     arguments=[LaunchConfiguration('map'), '5', '_frame_id:=map', 'cloud_pcd:=map']
    # )
    
    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        map_arg,
        fast_lio_node,
        rviz_node,
        pointcloud_to_map_launch,
        pointscloud_to_laserscan_launch,
        tf_pub_1,
        tf_pub_2
    ])
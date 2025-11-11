import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    fast_lio_dir = get_package_share_directory('fast_lio')
    fast_lio_localization_dir = get_package_share_directory('fast_lio_localization')
    
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
    
    map_2d_arg = DeclareLaunchArgument(
        '2dmap', 
        default_value='scans.yaml',
        description='2D地图YAML文件路径'
    )

    pcd_map_topic_arg = DeclareLaunchArgument(
        "pcd_map_topic", 
        default_value="cloud_pcd", 
        description="Topic to publish PCD map"
    )

    # fast_lio节点配置
    fast_lio_config = PathJoinSubstitution([
        fast_lio_localization_dir, 'config', 'mid360.yaml'
    ])
    
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[fast_lio_config]
    )
    
    # 全局定位节点
    global_localization_node = Node(
        package='fast_lio_localization',
        executable='global_localization.py',
        name='global_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[{"map_voxel_size": 0.4,
                     "scan_voxel_size": 0.1,
                     "freq_localization": 0.5,
                     "freq_global_map": 0.25,
                     "localization_threshold": 0.8,
                     "fov": 6.28319,
                     "fov_far": 300,
                     "pcd_map_path": LaunchConfiguration('map'),
                     "pcd_map_topic": LaunchConfiguration('pcd_map_topic')}],
    )

    
    # 变换融合节点
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
    )
    
    map_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": LaunchConfiguration('map'),
                     "tf_frame": "map",
                    "cloud_topic": LaunchConfiguration('pcd_map_topic'),
                    "period_ms_": 500}],
        remappings=[
            ("cloud_pcd", LaunchConfiguration('pcd_map_topic')),
        ]
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
                    fast_lio_localization_dir, 'rviz', 'fastlio_localization.rviz'
                ])],
                output='screen'
            )
        ]
    )
    
    # 2D地图服务器
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': PathJoinSubstitution([
                    fast_lio_dir, 'PCD', LaunchConfiguration('2dmap')
                ]),
                'topic_name': 'prior_map',
                'frame_id': 'map'
            }
        ]
    )
    
    # 包含PointsCloud2toLaserscan.launch
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
    
    # 被注释掉的静态TF发布（保持原样）
    # tf_pub_3 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='tf_pub_3',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']
    # )
    
    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,
        map_arg,
        map_2d_arg,
        pcd_map_topic_arg,
        fast_lio_node,
        global_localization_node,
        transform_fusion_node,
        map_publisher_node,
        rviz_node,
        map_server_node,
        pointscloud_to_laserscan_launch,
        tf_pub_1,
        tf_pub_2
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    package_path = get_package_share_directory("yhs_nav2_launcher")
    default_config_path = os.path.join(package_path, "config", "mid360_localization.yaml")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")
    default_2dmap_path = os.path.join(package_path, "maps", "map.yaml")
    default_3dmap_path = os.path.join(package_path, "maps", "pointcloud.pcd")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    map3d = LaunchConfiguration("map3d")
    map2d = LaunchConfiguration("map2d")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", default_value=default_config_path, description="Yaml config file path"
    )
    declare_rviz_cmd = DeclareLaunchArgument("rviz", default_value="true", description="Use RViz to monitor results")

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg", default_value=default_rviz_config_path, description="RViz config file path"
    )

    declare_2dmap_path = DeclareLaunchArgument("map2d", default_value=default_2dmap_path, description="Path to PCD map file")
    declare_3dmap_path = DeclareLaunchArgument("map3d", default_value=default_3dmap_path, description="Path to PCD map file")
    
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", default_value="cloud_pcd", description="Topic to publish PCD map"
    )
    # Load parameters from yaml file

    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Global localization node
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
                     "pcd_map_path": map3d,
                     "pcd_map_topic": pcd_map_topic}],
    )

    # Transform fusion node
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
    )
    
    # PCD to PointCloud2 publisher
    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": map3d,
                     "tf_frame": "map",
                    "cloud_topic": pcd_map_topic,
                    "period_ms_": 500}],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], condition=IfCondition(rviz_use))

    # # 2D地图服务器
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[
    #         {
    #             'yaml_filename': map2d,
    #             'topic_name': 'prior_map',
    #             'frame_id': 'map'
    #         }
    #     ]
    # )
    
    #pointscloud2 to laserscans
    laserscans_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([package_path,'/launch','/pointcloud_to_laserscan.launch.py'])
        )   

    # 静态TF发布

    tf_pub_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_2',
        arguments=['-0.30', '0', '0', '0', '0', '0', 'camera_init', 'robot_foot_init']
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_2dmap_path)
    ld.add_action(declare_3dmap_path)

    ld.add_action(declare_pcd_map_topic)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(pcd_publisher_node)

    # ld.add_action(map_server_node)
    # ld.add_action(laserscans_bringup_launch)
    # ld.add_action(tf_pub_1)
    # ld.add_action(tf_pub_2)


    return ld

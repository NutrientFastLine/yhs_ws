import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    launch_package_dir = get_package_share_directory('yhs_nav2_launcher')
    nav2_bringup_dir = get_package_share_directory('yhs_nav2_launcher')
    
    
    #=============================2.声明参数，获取配置文件路径===================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='False') 

    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(launch_package_dir,'maps','map.yaml'))
    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(launch_package_dir,'maps','toolbox_outside_2.yaml'))
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(launch_package_dir,'maps','map.yaml'))
    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(launch_package_dir,'maps','gazebo_1.yaml'))

    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(launch_package_dir,'config','nav2_params_Ackerman.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(launch_package_dir,'config','robot_outside_nav2.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(launch_package_dir,'config','robot_gazebo_nav2.yaml'))

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/navigation2/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    # trans_tf_2d_node = Node(
    #     package='yhs_nav2_launcher',
    #     executable='trans_tf_2d',  
    #     name='trans_tf_2d',        
    #     output='screen'            
    # )

    # trans_tf_2d_node = Node(
    #     package='yhs_nav2_launcher',
    #     executable='static_body_to_base_link',  
    #     name='static_body_to_base_link_publisher',        
    #     output='screen'            
    # )

    body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base',
        arguments=['-0.66', '0', '-0.40', '0', '0', '0', 'body', 'base_link'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d','/home/feiyu/yhs_ws/src/yhs_nav2_launcher/rviz/nav2_default_view.rviz'],
        output='screen'
    )   

    # return LaunchDescription([nav2_bringup_launch,trans_tf_2d_node,rviz_node])
    return LaunchDescription([nav2_bringup_launch,body_to_base,rviz_node])

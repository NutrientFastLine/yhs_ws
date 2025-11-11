import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    livox_lidar_dir = get_package_share_directory('turn_on_yhs_robot')
    livox_lidar_launch_dir = os.path.join(livox_lidar_dir, 'launch')

    pointcloud_filter_dir = get_package_share_directory('pointcloud_filter')
    pointcloud_filter_launch_dir = os.path.join(pointcloud_filter_dir, 'launch')

    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    pointcloud_to_laserscan_launch_dir = os.path.join(pointcloud_to_laserscan_dir, 'launch')
           
    MID360_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(livox_lidar_launch_dir, 'msg_MID360_launch.py')),)
    
    pointcloud_filter = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pointcloud_filter_launch_dir, 'filter.launch.py')),)
    
    pointcloud_to_scan = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pointcloud_to_laserscan_launch_dir, 'pointcloud_to_laserscan_launch.py')),)
                       
                  
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(MID360_launch)
    ld.add_action(pointcloud_filter)
    ld.add_action(pointcloud_to_scan)
    return ld


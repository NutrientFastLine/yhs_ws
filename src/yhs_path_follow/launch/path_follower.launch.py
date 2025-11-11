from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    # 路径文件参数
    path_filename = LaunchConfiguration('path_filename')
    declare_path_filename_arg = DeclareLaunchArgument(
        'path_filename',
        default_value='/home/feiyu/yhs_ws/src/yhs_path_follow/path/yhs_path',
        description='Name of the path file to follow'
    )
    
    # 坐标系参数
    path_frame = LaunchConfiguration('path_frame')
    declare_path_frame_arg = DeclareLaunchArgument(
        'path_frame',
        default_value='map',
        description='Frame ID for the path'
    )
    
    # 目标容差参数
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    declare_goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.1',
        description='Goal tolerance in meters'
    )
    
    # 前瞻距离参数
    path_lookahead_distance = LaunchConfiguration('path_lookahead_distance')
    declare_path_lookahead_distance_arg = DeclareLaunchArgument(
        'path_lookahead_distance',
        default_value='0.5',
        description='Lookahead distance for path following'
    )
    
    # 路径跟踪节点
    path_follower_node = Node(
        package='yhs_path_follow',
        executable='path_follower.py',
        name='path_follower',
        output='screen',
        parameters=[{
            'path_filename': path_filename,
            'path_frame': path_frame,
            'goal_tolerance': goal_tolerance,
            'path_lookahead_distance': path_lookahead_distance,
        }]
    )
    
    return LaunchDescription([
        # 声明参数
        declare_path_filename_arg,
        declare_path_frame_arg,
        declare_goal_tolerance_arg,
        declare_path_lookahead_distance_arg,
        
        # 启动节点
        path_follower_node,
    ])
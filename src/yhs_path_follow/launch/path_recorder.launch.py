from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 路径文件参数
    path_filename = LaunchConfiguration('path_filename')
    declare_path_filename_arg = DeclareLaunchArgument(
        'path_filename',
        default_value='/home/feiyu/yhs_ws/src/yhs_path_follow/path/yhs_path',
        description='Name of the file to save the recorded path'
    )
    
    # 最小距离参数
    min_distance = LaunchConfiguration('min_distance')
    declare_min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.05',
        description='Minimum distance between consecutive path points in meters'
    )
    
    # 最小角度参数
    min_angle = LaunchConfiguration('min_angle')
    declare_min_angle_arg = DeclareLaunchArgument(
        'min_angle',
        default_value='10.0',
        description='Minimum angle between consecutive path points in degrees'
    )
    
    # 父坐标系参数
    parent_frame = LaunchConfiguration('parent_frame')
    declare_parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='map',
        description='Parent frame for transform lookup'
    )
    
    # 子坐标系参数
    child_frame = LaunchConfiguration('child_frame')
    declare_child_frame_arg = DeclareLaunchArgument(
        'child_frame',
        default_value='base_link',
        description='Child frame for transform lookup'
    )
    
    # 更新频率参数
    update_frequency = LaunchConfiguration('update_frequency')
    declare_update_frequency_arg = DeclareLaunchArgument(
        'update_frequency',
        default_value='1.0',
        description='Frequency for path recording in Hz'
    )
    
    # 路径记录节点
    path_recorder_node = Node(
        package='yhs_path_follow',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[{
            'path_filename': path_filename,
            'min_distance': min_distance,
            'min_angle': min_angle,
            'parent_frame': parent_frame,
            'child_frame': child_frame,
            'update_frequency': update_frequency,
        }]
    )
    
    return LaunchDescription([
        # 声明参数
        declare_path_filename_arg,
        declare_min_distance_arg,
        declare_min_angle_arg,
        declare_parent_frame_arg,
        declare_child_frame_arg,
        declare_update_frequency_arg,
        
        # 启动节点
        path_recorder_node,
    ])
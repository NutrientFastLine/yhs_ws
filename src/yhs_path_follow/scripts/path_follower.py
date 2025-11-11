#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import math
import os
from tf_transformations import quaternion_from_euler

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        # 参数声明
        self.declare_parameter('path_filename', 'yhs_path')
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('path_lookahead_distance', 0.5)
        
        # 获取参数
        self.path_filename = self.get_parameter('path_filename').get_parameter_value().string_value
        self.path_frame = self.get_parameter('path_frame').get_parameter_value().string_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.path_lookahead_distance = self.get_parameter('path_lookahead_distance').get_parameter_value().double_value
        
        # 初始化导航器
        self.navigator = BasicNavigator()
        
        # 发布器
        self.path_pub = self.create_publisher(Path, 'path_to_follow', 10)
        self.status_pub = self.create_publisher(Bool, 'is_navigating', 10)
        
        # 服务
        self.start_following_service = self.create_service(
            Trigger, 
            'start_following_path', 
            self.start_following_callback
        )
        
        self.stop_following_service = self.create_service(
            Trigger, 
            'stop_following_path', 
            self.stop_following_callback
        )
        
        # 定时器
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.is_navigating = False
        
        # 等待Nav2系统就绪
        self.get_logger().info('Waiting for Nav2 system to become active...')
        # self.navigator.waitUntilNav2Active()
        
        self.get_logger().info('Path follower initialized')
        self.get_logger().info(f'Path file: {self.path_filename}')
        self.get_logger().info(f'Path frame: {self.path_frame}')
        self.get_logger().info('Use services to control:')
        self.get_logger().info('  ros2 service call /start_following_path std_srvs/srv/Trigger')
        self.get_logger().info('  ros2 service call /stop_following_path std_srvs/srv/Trigger')
        
        self.publish_navigation_status()

    def timer_callback(self):
        if not self.is_navigating:
            return
        
        # 检查导航是否完成
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Path following completed successfully!')
            elif result == TaskResult.CANCELED:
                self.get_logger().warning('Path following was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().error('Path following failed!')
            else:
                self.get_logger().info('Path following completed with unknown result')
            
            self.is_navigating = False
            self.publish_navigation_status()
        else:
            # 导航进行中，可以获取反馈信息
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().debug(
                    f'Navigation in progress... Distance remaining: {feedback.distance_to_goal:.2f}'
                )

    def start_following_callback(self, request, response):
        if self.is_navigating:
            response.success = False
            response.message = 'Already following a path'
            self.get_logger().warning('Already following a path')
            return response
        
        # 加载路径文件
        path = self.load_path_from_file()
        if len(path.poses) == 0:
            response.success = False
            response.message = f'Failed to load path from file: {self.path_filename}'
            self.get_logger().error(f'Failed to load path from file: {self.path_filename}')
            return response
        
        self.get_logger().info(f'Loaded path with {len(path.poses)} points')
        
        # 发布路径用于可视化
        self.path_pub.publish(path)
        
        # 开始跟踪路径
        try:
            # followPath 是非阻塞调用
            self.navigator.followPath(path)
            self.is_navigating = True
            self.publish_navigation_status()
            
            response.success = True
            response.message = f'Started following path with {len(path.poses)} points'
            self.get_logger().info(f'Started following path with {len(path.poses)} points')
        except Exception as e:
            response.success = False
            response.message = f'Failed to start path following: {str(e)}'
            self.get_logger().error(f'Failed to start path following: {str(e)}')
        
        return response

    def stop_following_callback(self, request, response):
        if not self.is_navigating:
            response.success = False
            response.message = 'Not currently following any path'
            self.get_logger().warning('Not currently following any path')
            return response
        
        try:
            self.navigator.cancelTask()
            self.is_navigating = False
            self.publish_navigation_status()
            
            response.success = True
            response.message = 'Path following stopped'
            self.get_logger().info('Path following stopped')
        except Exception as e:
            response.success = False
            response.message = f'Failed to stop path following: {str(e)}'
            self.get_logger().error(f'Failed to stop path following: {str(e)}')
        
        return response

    def load_path_from_file(self):
        path = Path()
        path.header.frame_id = self.path_frame
        path.header.stamp = self.get_clock().now().to_msg()
        
        full_path = self.path_filename
        if full_path[0] != '/':
            # 如果是相对路径，添加到home目录
            home_dir = os.getenv('HOME')
            if home_dir:
                full_path = os.path.join(home_dir, full_path)
        
        self.get_logger().info(f'Loading path from: {full_path}')
        
        try:
            with open(full_path, 'r') as file:
                lines = file.readlines()
        except Exception as e:
            self.get_logger().error(f'Failed to open path file: {full_path}, error: {str(e)}')
            return path
        
        point_count = 0
        for line in lines:
            data = line.strip().split()
            if len(data) < 3:
                continue
            
            try:
                x = float(data[0])
                y = float(data[1])
                yaw = float(data[2])
            except ValueError:
                continue
            
            pose = PoseStamped()
            pose.header.frame_id = self.path_frame
            pose.header.stamp = path.header.stamp
            
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # 将偏航角转换为四元数
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path.poses.append(pose)
            point_count += 1
            
            self.get_logger().debug(f'Loaded point {point_count}: ({x:.3f}, {y:.3f}, {yaw:.3f})')
        
        if len(path.poses) == 0:
            self.get_logger().warning('No valid points found in path file')
        else:
            self.get_logger().info(f'Successfully loaded {point_count} path points')
        
        return path

    def publish_navigation_status(self):
        msg = Bool()
        msg.data = self.is_navigating
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    
    node = PathFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
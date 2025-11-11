#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include <fstream>
#include <memory>

class PathRecorder : public rclcpp::Node
{
public:
    PathRecorder() : Node("path_recorder"), is_recording_(false)
    {
        // 参数声明
        this->declare_parameter<std::string>("path_filename", "yhs_path");
        this->declare_parameter<double>("min_distance", 0.05);
        this->declare_parameter<double>("min_angle", 10.0); // 度
        this->declare_parameter<std::string>("parent_frame", "map");
        this->declare_parameter<std::string>("child_frame", "base_link");
        this->declare_parameter<double>("update_frequency", 10.0); // Hz
        
        // 获取参数
        this->get_parameter("path_filename", path_filename_);
        this->get_parameter("min_distance", min_distance_);
        this->get_parameter("min_angle", min_angle_);
        this->get_parameter("parent_frame", parent_frame_);
        this->get_parameter("child_frame", child_frame_);
        this->get_parameter("update_frequency", update_frequency_);
        
        // 转换为弧度
        min_angle_rad_ = min_angle_ * M_PI / 180.0;
        
        // 发布器
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("recorded_path", 10);
        recording_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_recording_path", 10);
        
        // 服务 - 开始记录
        start_recording_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_recording_path",
            std::bind(&PathRecorder::startRecordingCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // 服务 - 保存路径（同时停止记录）
        save_path_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_recorded_path",
            std::bind(&PathRecorder::savePathCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 定时器 - 固定频率更新
        auto timer_period = std::chrono::duration<double>(1.0 / update_frequency_);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&PathRecorder::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "Path recorder initialized");
        RCLCPP_INFO(this->get_logger(), "Parent frame: %s, Child frame: %s", 
                   parent_frame_.c_str(), child_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Min distance: %.3f, Min angle: %.1f deg", 
                   min_distance_, min_angle_);
        RCLCPP_INFO(this->get_logger(), "Update frequency: %.1f Hz", update_frequency_);
        RCLCPP_INFO(this->get_logger(), "Use services to control:");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /start_recording_path std_srvs/srv/Trigger");
        RCLCPP_INFO(this->get_logger(), "  ros2 service call /save_recorded_path std_srvs/srv/Trigger");
        
        // 发布初始状态
        publishRecordingStatus();
    }

private:
    void timerCallback()
    {
        if (!is_recording_) {
            return;
        }
        
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                parent_frame_, child_frame_, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "Transform lookup failed: %s", ex.what());
            return;
        }
        
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.stamp = this->now();
        current_pose.header.frame_id = parent_frame_;
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.orientation = transform.transform.rotation;
        
        // 如果是第一个点，直接添加
        if (recorded_path_.poses.empty()) {
            recorded_path_.poses.push_back(current_pose);
            recorded_path_.header = current_pose.header;
            RCLCPP_INFO(this->get_logger(), "First point recorded: (%.3f, %.3f)", 
                       current_pose.pose.position.x, current_pose.pose.position.y);
            return;
        }
        
        // 检查距离和角度变化
        auto& last_pose = recorded_path_.poses.back();
        double distance = std::hypot(
            last_pose.pose.position.x - current_pose.pose.position.x,
            last_pose.pose.position.y - current_pose.pose.position.y);
            
        double last_yaw = tf2::getYaw(last_pose.pose.orientation);
        double current_yaw = tf2::getYaw(current_pose.pose.orientation);
        double angle_diff = std::abs(last_yaw - current_yaw);
        
        // 如果变化足够大，添加新点
        if (distance >= min_distance_ || angle_diff >= min_angle_rad_) {
            recorded_path_.poses.push_back(current_pose);
            RCLCPP_INFO(this->get_logger(), "New point recorded: (%.3f, %.3f, %.3f) - Total: %zu points", 
                       current_pose.pose.position.x, 
                       current_pose.pose.position.y,
                       current_yaw,
                       recorded_path_.poses.size());
        }
    }
    
    void startRecordingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (is_recording_) {
            response->success = false;
            response->message = "Already recording path";
            RCLCPP_WARN(this->get_logger(), "Already recording path");
            return;
        }
        
        // 清空之前的路径
        recorded_path_.poses.clear();
        recorded_path_.header.frame_id = parent_frame_;
        recorded_path_.header.stamp = this->now();
        
        is_recording_ = true;
        publishRecordingStatus();
        
        response->success = true;
        response->message = "Started recording path";
        RCLCPP_INFO(this->get_logger(), "Started recording path");
    }
    
    void savePathCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // 停止记录（如果正在记录）
        bool was_recording = is_recording_;
        if (is_recording_) {
            is_recording_ = false;
            publishRecordingStatus();
            RCLCPP_INFO(this->get_logger(), "Stopped recording path");
        }
        
        if (recorded_path_.poses.empty()) {
            response->success = false;
            response->message = "No path points to save";
            RCLCPP_WARN(this->get_logger(), "No path points to save");
            return;
        }
        
        std::string full_path = path_filename_;
        if (full_path[0] != '/') {
            // 如果是相对路径，添加到home目录
            const char* home_dir = std::getenv("HOME");
            if (home_dir) {
                full_path = std::string(home_dir) + "/" + full_path;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Saving path to: %s", full_path.c_str());
        
        std::ofstream out(full_path, std::ios_base::trunc);
        if (!out.is_open()) {
            response->success = false;
            response->message = "Failed to open file: " + full_path;
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", full_path.c_str());
            return;
        }
        
        // 保存路径点
        for (const auto& pose : recorded_path_.poses) {
            double yaw = tf2::getYaw(pose.pose.orientation);
            out << pose.pose.position.x << " " 
                << pose.pose.position.y << " " 
                << yaw << "\n";
        }
        
        out.close();
        
        // 发布完整路径
        recorded_path_.header.stamp = this->now();
        path_pub_->publish(recorded_path_);
        
        std::string message = "Path saved with " + std::to_string(recorded_path_.poses.size()) + 
                             " points to: " + full_path;
        if (was_recording) {
            message += " (recording was stopped)";
        }
        
        response->success = true;
        response->message = message;
        RCLCPP_INFO(this->get_logger(), "Path saved with %zu points to: %s", 
                   recorded_path_.poses.size(), full_path.c_str());
    }
    
    void publishRecordingStatus()
    {
        std_msgs::msg::Bool status_msg;
        status_msg.data = is_recording_;
        recording_status_pub_->publish(status_msg);
    }
    
    // 成员变量
    std::string path_filename_;
    double min_distance_;
    double min_angle_;
    double min_angle_rad_;
    std::string parent_frame_;
    std::string child_frame_;
    double update_frequency_;
    
    bool is_recording_;
    nav_msgs::msg::Path recorded_path_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recording_status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_recording_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_path_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
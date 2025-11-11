#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TransTf2d : public rclcpp::Node
{
public:
  TransTf2d() : Node("Trans_TF_2d")
  {
    // 创建TF监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 创建定时器，替代ROS1中的ros::Rate
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),  // 1ms，接近原代码的1000Hz
      std::bind(&TransTf2d::timerCallback, this));
    
    // 初始休眠，替代ROS1中的ros::Duration(1.0).sleep()
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

private:
  void timerCallback()
  {
    try {
      // 查找变换
      auto transform = tf_buffer_->lookupTransform(
        "map", "body", tf2::TimePointZero);
      
      // 获取位置和姿态
      // double robot_pose_x = transform.transform.translation.x;
      // double robot_pose_y = transform.transform.translation.y;
      double robot_oriation_z = transform.transform.rotation.z;
      double robot_oriation_w = transform.transform.rotation.w;
      
      // 创建新的变换
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = this->get_clock()->now();
      transform_stamped.header.frame_id = "body";
      transform_stamped.child_frame_id = "base_link";
      
      transform_stamped.transform.translation.x = -0.66;
      transform_stamped.transform.translation.y = -0.40;
      transform_stamped.transform.translation.z = 0.0;
      
      transform_stamped.transform.rotation.x = 0.0;
      transform_stamped.transform.rotation.y = 0.0;
      transform_stamped.transform.rotation.z = robot_oriation_z;
      transform_stamped.transform.rotation.w = robot_oriation_w;
      
      // 广播变换
      tf_broadcaster_->sendTransform(transform_stamped);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransTf2d>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticBodyToBaseLinkPublisher : public rclcpp::Node
{
public:
  StaticBodyToBaseLinkPublisher()
  : Node("static_body_to_base_link_publisher")
  {
    // 创建静态TF广播器
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 定义静态变换
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "body";
    transformStamped.child_frame_id = "base_link";

    // 平移部分
    transformStamped.transform.translation.x = -0.66;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = -0.4;

    // 旋转部分（四元数）
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    // 发送静态TF
    static_broadcaster_->sendTransform(transformStamped);

    RCLCPP_INFO(this->get_logger(), "Published static transform body -> base_link (all zeros)");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticBodyToBaseLinkPublisher>());
  rclcpp::shutdown();
  return 0;
}

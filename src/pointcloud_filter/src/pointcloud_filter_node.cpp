#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <vector>

class LivoxPointCloudFilterNode : public rclcpp::Node {
public:
  LivoxPointCloudFilterNode() : Node("livox_pointcloud_filter_node") {
    // 声明可配置参数（带默认值）
    this->declare_parameter<std::vector<double>>("robot_min", {-0.70, -0.2325, -0.2});
    this->declare_parameter<std::vector<double>>("robot_max", {0.10, 0.2325, 0.8});
    this->declare_parameter<double>("rear_angle_threshold", 150.0); // 单位：度
    this->declare_parameter<bool>("enable_angle_filter", true); 

    // 获取参数值
    auto robot_min = this->get_parameter("robot_min").as_double_array();
    auto robot_max = this->get_parameter("robot_max").as_double_array();
    rear_angle_threshold_ = this->get_parameter("rear_angle_threshold").as_double();
    enable_angle_filter_ = this->get_parameter("enable_angle_filter").as_bool();

    // 检查参数有效性
    if (robot_min.size() != 3 || robot_max.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "Invalid robot_min/robot_max size. Using defaults.");
      robot_min_ = {-0.70, -0.2325, -0.2};
      robot_max_ = {0.10, 0.2325, 0.8};
    } else {
      robot_min_ = {robot_min[0], robot_min[1], robot_min[2]};
      robot_max_ = {robot_max[0], robot_max[1], robot_max[2]};
    }
    
    RCLCPP_INFO(this->get_logger(), "Robot bounding box: Min(%.3f, %.3f, %.3f) Max(%.3f, %.3f, %.3f)",
                robot_min_[0], robot_min_[1], robot_min_[2],
                robot_max_[0], robot_max_[1], robot_max_[2]);
    RCLCPP_INFO(this->get_logger(), "Rear angle threshold: %.1f degrees", rear_angle_threshold_);

    // 订阅发布话题
    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", rclcpp::SensorDataQoS(),
      std::bind(&LivoxPointCloudFilterNode::pointcloud_callback, this, std::placeholders::_1));

    custom_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/msg_filtered", 10);
    pc2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2_filtered", 10);
  }

private:
  void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    std::vector<livox_ros_driver2::msg::CustomPoint> filtered_points;
    filtered_points.reserve(msg->points.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (const auto& pt : msg->points) {
      // 使用参数化的机器人边界框过滤
      if (pt.x >= robot_min_[0] && pt.x <= robot_max_[0] &&
          pt.y >= robot_min_[1] && pt.y <= robot_max_[1] &&
          pt.z >= robot_min_[2] && pt.z <= robot_max_[2]) {
        continue;
      }
      
      // 使用参数化的后方角度阈值过滤
      if (enable_angle_filter_) {
        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < -rear_angle_threshold_ || angle > rear_angle_threshold_) {
          continue;
        }
      }
      
      filtered_points.push_back(pt);
      
      pcl::PointXYZI pcl_point;
      pcl_point.x = pt.x;
      pcl_point.y = pt.y;
      pcl_point.z = pt.z;
      pcl_point.intensity = static_cast<float>(pt.reflectivity);
      pcl_cloud->points.push_back(pcl_point);
    }

    // 发布CustomMsg格式
    auto custom_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    custom_msg->header = msg->header;
    custom_msg->timebase = msg->timebase;
    custom_msg->point_num = static_cast<uint32_t>(filtered_points.size());
    custom_msg->lidar_id = msg->lidar_id;
    custom_msg->rsvd = msg->rsvd;
    custom_msg->points = std::move(filtered_points);
    custom_publisher_->publish(*custom_msg);

    // 发布PointCloud2格式
    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
    
    sensor_msgs::msg::PointCloud2 pc2_msg;
    pcl::toROSMsg(*pcl_cloud, pc2_msg);
    pc2_msg.header = msg->header;
    pc2_publisher_->publish(pc2_msg);
  }

  // 参数变量
  std::vector<double> robot_min_;
  std::vector<double> robot_max_;
  double rear_angle_threshold_;
  bool enable_angle_filter_; 

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxPointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
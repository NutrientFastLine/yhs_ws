#define _USE_MATH_DEFINES  // 确保在#include <cmath>之前
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"

// 节点类：实现Nav2速度指令到底盘指令的转换
class Nav2ChassisAdapter : public rclcpp::Node
{
public:
    Nav2ChassisAdapter() : Node("nav2_chassis_adapter")
    {
        // 1. 创建发布者：发布底盘控制指令（CtrlCmd）
        chassis_cmd_pub_ = this->create_publisher<yhs_can_interfaces::msg::CtrlCmd>(
            "/ctrl_cmd", 10); 

        // 2. 创建订阅者：订阅Nav2发布的速度指令（Twist）
        nav2_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Nav2ChassisAdapter::twist_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nav2-Chassis adapter node initialized.");
    }

private:
    // 订阅回调：处理Nav2的Twist消息，转换为CtrlCmd并发布
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {
        // 初始化底盘控制指令
        yhs_can_interfaces::msg::CtrlCmd chassis_cmd;

        // --------------------------
        // 核心转换逻辑（需根据车辆参数调整）
        // --------------------------

        // 1. 速度指令（linear.x对应前进速度，单位：m/s）
        // 注意：示例中限制了反向速度（参考你的回调逻辑）
        float vel_mps = twist_msg->linear.x;
        chassis_cmd.ctrl_cmd_velocity = vel_mps;  // 直接使用m/s

        // 2. 转向指令（angular.z对应角速度，单位：rad/s -> 转换为 °/s）
        float steer_rad = twist_msg->angular.z;
        // 弧度转角度：弧度 × (180/π) = 角度
        float steer_deg = steer_rad * (180.0 / M_PI);
        chassis_cmd.ctrl_cmd_steering = steer_deg;

        // 3. 档位指令（根据速度判断：前进/空挡/后退）
        if (vel_mps > 0)
        {
            chassis_cmd.ctrl_cmd_gear = 4;  // 1：前进档
        }
        else if (vel_mps < 0)
        {
            chassis_cmd.ctrl_cmd_velocity = -vel_mps;  // 取正直接使用m/s
            chassis_cmd.ctrl_cmd_gear = 2;  // 2：后退档
        }
        else
        {
            chassis_cmd.ctrl_cmd_gear = 3;  // 0：空挡
        }

        // 4. 刹车指令（无速度时刹车，或根据需求自定义）
        // if (vel_mps == 0 && std::abs(steer_rad) == 0)
        // {
        //     chassis_cmd.ctrl_cmd_brake = 1;  // 1：轻刹车（需与底盘定义一致）
        // }
        // else
        // {
        //     chassis_cmd.ctrl_cmd_brake = 0;  // 0：不刹车
        // }

        // 发布转换后的底盘指令
        chassis_cmd_pub_->publish(chassis_cmd);
        RCLCPP_DEBUG(this->get_logger(), "Converted cmd: vel=%.2f, steer=%.2f, gear=%d, brake=%d",
                    chassis_cmd.ctrl_cmd_velocity,
                    chassis_cmd.ctrl_cmd_steering,
                    chassis_cmd.ctrl_cmd_gear,
                    chassis_cmd.ctrl_cmd_brake);
    }

    // 成员变量：发布者和订阅者
    rclcpp::Publisher<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr chassis_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_vel_sub_;
};

int main(int argc, char * argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    // 运行节点
    rclcpp::spin(std::make_shared<Nav2ChassisAdapter>());
    // 退出
    rclcpp::shutdown();
    return 0;
}
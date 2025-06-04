#ifndef CARLA_CONTROL_ADAPTER_HPP_
#define CARLA_CONTROL_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include "auto_msgs/msg/vehicle_control.hpp"

// CARLA messages (commented out for now)
// #include "carla_msgs/msg/carla_ego_vehicle_control.hpp"

// 临时控制消息结构（用于替代 CARLA 消息）
struct TempCarlaControl {
    float steer = 0.0f;
    float throttle = 0.0f;
    float brake = 0.0f;
    bool hand_brake = false;
    bool reverse = false;
    bool manual_gear_shift = false;
    int32_t gear = 1;
};

namespace carla_integration {

/**
 * @brief CARLA 控制适配器
 * 
 * 该类负责接收自动驾驶控制指令并转换为 CARLA 车辆控制命令
 * 功能：
 * 1. 订阅标准化的车辆控制指令
 * 2. 转换为 CARLA 特定的控制格式
 * 3. 发布给 CARLA 车辆
 * 4. 安全监控和限制
 */
class CarlaControlAdapter : public rclcpp::Node {
public:
    CarlaControlAdapter();

private:
    /**
     * @brief 车辆控制指令回调函数
     * @param msg 车辆控制消息
     */
    void vehicleControlCallback(const auto_msgs::msg::VehicleControl::SharedPtr msg);
    
    /**
     * @brief Twist 控制指令回调函数
     * @param msg Twist 消息
     */
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    /**
     * @brief 转换车辆控制指令为临时控制格式
     * @param control_msg 标准控制消息
     * @return 临时控制消息
     */
    TempCarlaControl convertToCarlaControl(
        const auto_msgs::msg::VehicleControl& control_msg);
    
    /**
     * @brief 从 Twist 消息转换为临时控制
     * @param twist_msg Twist 消息
     * @return 临时控制消息
     */
    TempCarlaControl convertTwistToCarlaControl(
        const geometry_msgs::msg::Twist& twist_msg);
    
    /**
     * @brief 安全限制检查
     * @param control 临时控制消息
     * @return 限制后的控制消息
     */
    TempCarlaControl applySafetyLimits(
        const TempCarlaControl& control);
    
    /**
     * @brief 控制超时检查
     */
    void checkControlTimeout();
    
    /**
     * @brief 发送紧急制动
     */
    void sendEmergencyBrake();

    // ROS 2 订阅者
    rclcpp::Subscription<auto_msgs::msg::VehicleControl>::SharedPtr vehicle_control_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    
    // ROS 2 发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr carla_control_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    
    // 最后接收控制指令的时间
    rclcpp::Time last_control_time_;
    
    // 参数
    double max_steering_angle_;
    double max_throttle_;
    double max_brake_;
    double control_timeout_;
    double emergency_brake_threshold_;
    
    // 状态
    bool emergency_stop_;
};

} // namespace carla_integration

#endif // CARLA_CONTROL_ADAPTER_HPP_

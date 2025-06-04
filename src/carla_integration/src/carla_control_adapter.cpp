#include "carla_integration/carla_control_adapter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace carla_integration {

CarlaControlAdapter::CarlaControlAdapter() 
    : Node("carla_control_adapter"), emergency_stop_(false) {
    
    // 声明参数
    this->declare_parameter("max_steering_angle", 0.7);
    this->declare_parameter("max_throttle", 1.0);
    this->declare_parameter("max_brake", 1.0);
    this->declare_parameter("control_timeout", 0.5);
    this->declare_parameter("emergency_brake_threshold", 0.1);
    
    // 获取参数
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    max_throttle_ = this->get_parameter("max_throttle").as_double();
    max_brake_ = this->get_parameter("max_brake").as_double();
    control_timeout_ = this->get_parameter("control_timeout").as_double();
    emergency_brake_threshold_ = this->get_parameter("emergency_brake_threshold").as_double();
    
    RCLCPP_INFO(this->get_logger(), "CARLA Control Adapter initialized");
    RCLCPP_INFO(this->get_logger(), "Max steering: %.2f rad, Max throttle: %.2f, Max brake: %.2f", 
                max_steering_angle_, max_throttle_, max_brake_);
    RCLCPP_INFO(this->get_logger(), "Control timeout: %.2f s", control_timeout_);
    
    // 创建订阅者
    vehicle_control_sub_ = this->create_subscription<auto_msgs::msg::VehicleControl>(
        "/control/vehicle_control", 10,
        std::bind(&CarlaControlAdapter::vehicleControlCallback, this, std::placeholders::_1));
    
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&CarlaControlAdapter::twistCallback, this, std::placeholders::_1));
    
    // 创建发布者
    carla_control_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/vehicle_control_cmd", 10);
    
    // 创建超时检查定时器
    auto timer_period = std::chrono::milliseconds(100);  // 10Hz 检查频率
    timeout_timer_ = this->create_wall_timer(
        timer_period, std::bind(&CarlaControlAdapter::checkControlTimeout, this));
    
    // 初始化时间
    last_control_time_ = this->get_clock()->now();
}

void CarlaControlAdapter::vehicleControlCallback(const auto_msgs::msg::VehicleControl::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received vehicle control command");
    
    // 更新最后控制时间
    last_control_time_ = this->get_clock()->now();
    emergency_stop_ = false;
    
    // 转换为临时控制格式
    auto carla_control = convertToCarlaControl(*msg);
    
    // 应用安全限制
    carla_control = applySafetyLimits(carla_control);
    
    // 发布控制指令 (转换为字符串消息)
    std_msgs::msg::String control_str;
    control_str.data = "steer:" + std::to_string(carla_control.steer) + 
                      ",throttle:" + std::to_string(carla_control.throttle) +
                      ",brake:" + std::to_string(carla_control.brake);
    carla_control_pub_->publish(control_str);
}

void CarlaControlAdapter::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received twist command");
    
    // 更新最后控制时间
    last_control_time_ = this->get_clock()->now();
    emergency_stop_ = false;
    
    // 转换为临时控制格式
    auto carla_control = convertTwistToCarlaControl(*msg);
    
    // 应用安全限制
    carla_control = applySafetyLimits(carla_control);
    
    // 发布控制指令 (转换为字符串消息)
    std_msgs::msg::String control_str;
    control_str.data = "steer:" + std::to_string(carla_control.steer) + 
                      ",throttle:" + std::to_string(carla_control.throttle) +
                      ",brake:" + std::to_string(carla_control.brake);
    carla_control_pub_->publish(control_str);
}

TempCarlaControl CarlaControlAdapter::convertToCarlaControl(
    const auto_msgs::msg::VehicleControl& control_msg) {
    
    TempCarlaControl carla_control;
    
    // 转向角度 (弧度转换为归一化值 -1 到 1)
    carla_control.steer = std::clamp(control_msg.steer / max_steering_angle_, -1.0f, 1.0f);
    
    // 油门和制动
    if (control_msg.throttle >= 0) {
        carla_control.throttle = std::clamp(static_cast<float>(control_msg.throttle), 0.0f, 1.0f);
        carla_control.brake = 0.0f;
    } else {
        carla_control.throttle = 0.0f;
        carla_control.brake = std::clamp(static_cast<float>(-control_msg.throttle), 0.0f, 1.0f);
    }
    
    // 手刹和档位
    carla_control.hand_brake = control_msg.hand_brake;
    carla_control.reverse = control_msg.reverse;
    carla_control.manual_gear_shift = false;
    carla_control.gear = 1;  // 自动档
    
    return carla_control;
}

TempCarlaControl CarlaControlAdapter::convertTwistToCarlaControl(
    const geometry_msgs::msg::Twist& twist_msg) {
    
    TempCarlaControl carla_control;
    
    // 从线速度和角速度计算转向角
    double linear_velocity = twist_msg.linear.x;
    double angular_velocity = twist_msg.angular.z;
    
    // 简单的阿克曼转向模型 (假设轴距为 2.5m)
    double wheelbase = 2.5;
    double steering_angle = 0.0;
    
    if (std::abs(linear_velocity) > 0.1) {
        steering_angle = std::atan(angular_velocity * wheelbase / linear_velocity);
    }
    
    carla_control.steer = std::clamp(static_cast<float>(steering_angle / max_steering_angle_), -1.0f, 1.0f);
    
    // 速度控制
    if (linear_velocity > 0) {
        carla_control.throttle = std::clamp(static_cast<float>(linear_velocity / 10.0), 0.0f, 1.0f);  // 假设最大速度 10 m/s
        carla_control.brake = 0.0f;
        carla_control.reverse = false;
    } else if (linear_velocity < 0) {
        carla_control.throttle = std::clamp(static_cast<float>(-linear_velocity / 5.0), 0.0f, 1.0f);  // 倒车较慢
        carla_control.brake = 0.0f;
        carla_control.reverse = true;
    } else {
        carla_control.throttle = 0.0f;
        carla_control.brake = 0.3f;  // 轻微制动保持静止
        carla_control.reverse = false;
    }
    
    carla_control.hand_brake = false;
    carla_control.manual_gear_shift = false;
    carla_control.gear = 1;
    
    return carla_control;
}

TempCarlaControl CarlaControlAdapter::applySafetyLimits(
    const TempCarlaControl& control) {
    
    auto safe_control = control;
    
    // 转向角限制
    safe_control.steer = std::clamp(control.steer, -1.0f, 1.0f);
    
    // 油门限制
    safe_control.throttle = std::clamp(control.throttle, 0.0f, static_cast<float>(max_throttle_));
    
    // 制动限制
    safe_control.brake = std::clamp(control.brake, 0.0f, static_cast<float>(max_brake_));
    
    // 紧急停止检查
    if (emergency_stop_) {
        safe_control.throttle = 0.0f;
        safe_control.brake = 1.0f;
        safe_control.steer = 0.0f;
    }
    
    return safe_control;
}

void CarlaControlAdapter::checkControlTimeout() {
    auto current_time = this->get_clock()->now();
    auto time_diff = (current_time - last_control_time_).seconds();
    
    if (time_diff > control_timeout_) {
        if (!emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "Control timeout detected! Applying emergency brake.");
            sendEmergencyBrake();
            emergency_stop_ = true;
        }
    }
}

void CarlaControlAdapter::sendEmergencyBrake() {
    TempCarlaControl emergency_control;
    
    emergency_control.steer = 0.0f;
    emergency_control.throttle = 0.0f;
    emergency_control.brake = 1.0f;
    emergency_control.hand_brake = true;
    emergency_control.reverse = false;
    emergency_control.manual_gear_shift = false;
    emergency_control.gear = 1;
    
    // 发布控制指令 (转换为字符串消息)
    std_msgs::msg::String control_str;
    control_str.data = "steer:" + std::to_string(emergency_control.steer) + 
                      ",throttle:" + std::to_string(emergency_control.throttle) +
                      ",brake:" + std::to_string(emergency_control.brake) +
                      ",hand_brake:1";
    carla_control_pub_->publish(control_str);
    RCLCPP_WARN(this->get_logger(), "Emergency brake applied!");
}

} // namespace carla_integration

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<carla_integration::CarlaControlAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

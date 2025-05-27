#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

namespace auto_parking::entities {

/**
 * @brief 车辆状态实体 - Clean Architecture Entities层
 * 
 * 包含车辆的完整物理状态信息，用于自动泊车仿真
 * 遵循Clean Architecture原则，不依赖外部框架
 * 
 * 参考: Clean Architecture - https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html
 */
class VehicleState {
public:
    // 构造函数
    VehicleState() = default;
    
    // 位置和姿态
    struct Pose {
        double x{0.0};          // X坐标 (米)
        double y{0.0};          // Y坐标 (米)
        double theta{0.0};      // 航向角 (弧度)
    } pose;
    
    // 速度信息
    struct Velocity {
        double linear_x{0.0};   // 纵向速度 (m/s)
        double linear_y{0.0};   // 横向速度 (m/s)
        double angular_z{0.0};  // 角速度 (rad/s)
    } velocity;
    
    // 车辆控制输入
    struct Control {
        double steering_angle{0.0};     // 转向角 (弧度)
        double throttle{0.0};           // 油门 [0.0, 1.0]
        double brake{0.0};              // 刹车 [0.0, 1.0]
        bool reverse{false};            // 倒车档
    } control;
    
    // 车辆物理参数
    struct PhysicalParams {
        double wheelbase{2.7};          // 轴距 (米)
        double width{1.8};              // 车宽 (米)
        double length{4.5};             // 车长 (米)
        double max_steering_angle{0.6};  // 最大转向角 (弧度, ~35度)
        double max_speed{15.0};         // 最大速度 (m/s)
        double max_acceleration{3.0};   // 最大加速度 (m/s²)
        double max_deceleration{5.0};   // 最大减速度 (m/s²)
    } params;
    
    // 传感器状态
    struct Sensors {
        bool front_obstacle{false};     // 前方障碍物
        bool rear_obstacle{false};      // 后方障碍物
        bool left_obstacle{false};      // 左侧障碍物
        bool right_obstacle{false};     // 右侧障碍物
        double front_distance{10.0};    // 前方距离 (米)
        double rear_distance{10.0};     // 后方距离 (米)
        double left_distance{10.0};     // 左侧距离 (米)
        double right_distance{10.0};    // 右侧距离 (米)
    } sensors;
    
    // 时间戳
    std::chrono::steady_clock::time_point timestamp{std::chrono::steady_clock::now()};
    
    // 业务方法
    bool is_moving() const {
        return std::abs(velocity.linear_x) > 0.01 || std::abs(velocity.angular_z) > 0.01;
    }
    
    bool is_reversing() const {
        return control.reverse && velocity.linear_x < -0.01;
    }
    
    double get_speed() const {
        return std::sqrt(velocity.linear_x * velocity.linear_x + velocity.linear_y * velocity.linear_y);
    }
    
    bool has_obstacles() const {
        return sensors.front_obstacle || sensors.rear_obstacle || 
               sensors.left_obstacle || sensors.right_obstacle;
    }
    
    // 验证状态有效性
    bool is_valid() const {
        return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta) &&
               std::isfinite(velocity.linear_x) && std::isfinite(velocity.linear_y) && 
               std::isfinite(velocity.angular_z) &&
               control.throttle >= 0.0 && control.throttle <= 1.0 &&
               control.brake >= 0.0 && control.brake <= 1.0 &&
               std::abs(control.steering_angle) <= params.max_steering_angle;
    }
    
    // 更新时间戳
    void update_timestamp() {
        timestamp = std::chrono::steady_clock::now();
    }
};

} // namespace auto_parking::entities 
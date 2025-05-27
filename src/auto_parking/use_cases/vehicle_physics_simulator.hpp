#pragma once

#include "../entities/vehicle_state.hpp"
#include <cmath>
#include <algorithm>

namespace auto_parking::use_cases {

/**
 * @brief 车辆物理仿真器 - Clean Architecture Use Cases层
 * 
 * 实现基于自行车模型的车辆物理仿真，用于自动泊车
 * 包含转向、加速、制动等真实车辆动力学
 * 
 * 参考: 
 * - Bicycle Model: https://en.wikipedia.org/wiki/Bicycle_model
 * - Clean Architecture: https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html
 * 注意：AI自动生成，请人工审阅以防止可能的逻辑错误或幻觉现象。
 */
class VehiclePhysicsSimulator {
public:
    VehiclePhysicsSimulator() = default;
    
    /**
     * @brief 更新车辆物理状态
     * @param state 当前车辆状态
     * @param dt 时间步长 (秒)
     * @return 更新后的车辆状态
     */
    entities::VehicleState update_physics(const entities::VehicleState& state, double dt) {
        if (dt <= 0.0 || dt > 0.1) { // 限制时间步长
            return state;
        }
        
        entities::VehicleState new_state = state;
        
        // 1. 计算控制输入的实际效果
        double actual_throttle = std::clamp(state.control.throttle, 0.0, 1.0);
        double actual_brake = std::clamp(state.control.brake, 0.0, 1.0);
        double actual_steering = std::clamp(state.control.steering_angle, 
                                          -state.params.max_steering_angle, 
                                           state.params.max_steering_angle);
        
        // 2. 计算纵向力
        double longitudinal_force = calculate_longitudinal_force(state, actual_throttle, actual_brake);
        
        // 3. 计算加速度
        double acceleration = longitudinal_force / vehicle_mass_;
        
        // 4. 更新速度 (考虑倒车)
        double speed_direction = state.control.reverse ? -1.0 : 1.0;
        new_state.velocity.linear_x += acceleration * dt * speed_direction;
        
        // 5. 应用速度限制
        new_state.velocity.linear_x = std::clamp(new_state.velocity.linear_x, 
                                               -state.params.max_speed, 
                                                state.params.max_speed);
        
        // 6. 计算角速度 (自行车模型)
        if (std::abs(new_state.velocity.linear_x) > 0.01) {
            new_state.velocity.angular_z = (new_state.velocity.linear_x * std::tan(actual_steering)) / 
                                         state.params.wheelbase;
        } else {
            new_state.velocity.angular_z = 0.0;
        }
        
        // 7. 更新位置和姿态
        new_state.pose.x += new_state.velocity.linear_x * std::cos(state.pose.theta) * dt;
        new_state.pose.y += new_state.velocity.linear_x * std::sin(state.pose.theta) * dt;
        new_state.pose.theta += new_state.velocity.angular_z * dt;
        
        // 8. 规范化角度到 [-π, π]
        new_state.pose.theta = normalize_angle(new_state.pose.theta);
        
        // 9. 应用阻力和摩擦
        apply_resistance(new_state, dt);
        
        // 10. 更新控制状态
        new_state.control = state.control;
        new_state.control.steering_angle = actual_steering;
        
        // 11. 更新时间戳
        new_state.update_timestamp();
        
        return new_state;
    }
    
    /**
     * @brief 检查车辆是否可以执行给定的控制输入
     * @param state 当前车辆状态
     * @param control 期望的控制输入
     * @return 是否可以执行
     */
    bool can_execute_control(const entities::VehicleState& state, 
                           const entities::VehicleState::Control& control) const {
        // 检查转向角限制
        if (std::abs(control.steering_angle) > state.params.max_steering_angle) {
            return false;
        }
        
        // 检查油门和刹车范围
        if (control.throttle < 0.0 || control.throttle > 1.0 ||
            control.brake < 0.0 || control.brake > 1.0) {
            return false;
        }
        
        // 检查是否同时踩油门和刹车
        if (control.throttle > 0.1 && control.brake > 0.1) {
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 计算车辆在给定时间后的预测位置
     * @param state 当前状态
     * @param control 控制输入
     * @param prediction_time 预测时间
     * @param time_step 时间步长
     * @return 预测的车辆状态
     */
    entities::VehicleState predict_future_state(const entities::VehicleState& state,
                                               const entities::VehicleState::Control& control,
                                               double prediction_time,
                                               double time_step = 0.05) const {
        entities::VehicleState predicted_state = state;
        predicted_state.control = control;
        
        double elapsed_time = 0.0;
        while (elapsed_time < prediction_time) {
            double dt = std::min(time_step, prediction_time - elapsed_time);
            predicted_state = const_cast<VehiclePhysicsSimulator*>(this)->update_physics(predicted_state, dt);
            elapsed_time += dt;
        }
        
        return predicted_state;
    }
    
    /**
     * @brief 计算车辆转弯半径
     * @param state 车辆状态
     * @return 转弯半径 (米)，直行时返回无穷大
     */
    double calculate_turning_radius(const entities::VehicleState& state) const {
        if (std::abs(state.control.steering_angle) < 1e-6) {
            return std::numeric_limits<double>::infinity();
        }
        
        return state.params.wheelbase / std::tan(std::abs(state.control.steering_angle));
    }
    
    /**
     * @brief 计算停车距离
     * @param current_speed 当前速度 (m/s)
     * @param brake_force 制动力 [0.0, 1.0]
     * @return 停车距离 (米)
     */
    double calculate_stopping_distance(double current_speed, double brake_force = 1.0) const {
        if (current_speed <= 0.0 || brake_force <= 0.0) {
            return 0.0;
        }
        
        double max_deceleration = 5.0; // m/s²
        double actual_deceleration = max_deceleration * brake_force;
        
        // v² = u² + 2as, where v=0 (final speed), u=current_speed, a=-deceleration
        return (current_speed * current_speed) / (2.0 * actual_deceleration);
    }
    
    // 设置物理参数
    void set_vehicle_mass(double mass) { vehicle_mass_ = mass; }
    void set_drag_coefficient(double drag) { drag_coefficient_ = drag; }
    void set_rolling_resistance(double resistance) { rolling_resistance_ = resistance; }
    
private:
    // 物理参数
    double vehicle_mass_{1500.0};          // 车辆质量 (kg)
    double drag_coefficient_{0.3};         // 空气阻力系数
    double rolling_resistance_{0.015};     // 滚动阻力系数
    double gravity_{9.81};                  // 重力加速度 (m/s²)
    
    /**
     * @brief 计算纵向力
     */
    double calculate_longitudinal_force(const entities::VehicleState& state, 
                                      double throttle, double brake) const {
        double max_drive_force = 3000.0; // N
        double max_brake_force = 8000.0; // N
        
        double drive_force = throttle * max_drive_force;
        double brake_force = brake * max_brake_force;
        
        // 计算阻力
        double speed = std::abs(state.velocity.linear_x);
        double air_resistance = 0.5 * drag_coefficient_ * 1.225 * speed * speed; // 空气阻力
        double rolling_resistance_force = rolling_resistance_ * vehicle_mass_ * gravity_; // 滚动阻力
        
        double total_resistance = air_resistance + rolling_resistance_force;
        
        // 计算净力
        double net_force;
        if (brake > 0.01) {
            // 制动时
            net_force = -brake_force - total_resistance;
        } else {
            // 驱动时
            net_force = drive_force - total_resistance;
        }
        
        return net_force;
    }
    
    /**
     * @brief 应用阻力和摩擦
     */
    void apply_resistance(entities::VehicleState& state, double dt) const {
        // 应用速度衰减 (模拟摩擦)
        double friction_coefficient = 0.95; // 每秒保持95%的速度
        double friction_factor = std::pow(friction_coefficient, dt);
        
        if (std::abs(state.velocity.linear_x) < 0.1) {
            // 低速时增加摩擦，帮助车辆停止
            friction_factor *= 0.9;
        }
        
        state.velocity.linear_x *= friction_factor;
        state.velocity.angular_z *= friction_factor;
        
        // 极低速度时直接停止
        if (std::abs(state.velocity.linear_x) < 0.01) {
            state.velocity.linear_x = 0.0;
        }
        if (std::abs(state.velocity.angular_z) < 0.01) {
            state.velocity.angular_z = 0.0;
        }
    }
    
    /**
     * @brief 规范化角度到 [-π, π]
     */
    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace auto_parking::use_cases 
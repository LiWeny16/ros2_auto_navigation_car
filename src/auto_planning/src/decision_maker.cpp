#include "auto_planning/decision_maker.hpp"
#include <cmath>
#include <algorithm>
#include "tf2/utils.h"
#include "rclcpp/rclcpp.hpp"

namespace auto_planning {

DecisionMaker::DecisionMaker()
    : safety_distance_(5.0),
      emergency_distance_(2.0) {}

DecisionResult DecisionMaker::makeDecision(
    const std::vector<auto_perception::DetectedObject>& detected_objects,
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& goal_pose,
    bool has_path,
    double current_speed) {
    
    // 计算当前朝向角
    double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    
    // 检查前方是否有障碍物
    double look_ahead_distance = std::max(10.0, current_speed * 3.0);  // 根据速度调整前视距离
    auto [has_obstacle, distance] = checkObstaclesAhead(
        detected_objects, current_pose, current_yaw, look_ahead_distance, 4.0);
    
    // 如果前方有障碍物
    if (has_obstacle) {
        // 如果距离小于紧急停车距离
        if (distance < emergency_distance_) {
            return DecisionResult(
                DecisionType::EMERGENCY_STOP,
                "前方有紧急障碍物",
                distance);
        }
        // 如果距离小于安全距离
        else if (distance < safety_distance_) {
            return DecisionResult(
                DecisionType::STOP,
                "前方有障碍物",
                distance);
        }
        // 如果有路径但障碍物在路径上
        else if (has_path) {
            return DecisionResult(
                DecisionType::REPLAN,
                "路径上有障碍物",
                distance);
        }
    }
    
    // 如果没有路径，需要规划
    if (!has_path) {
        return DecisionResult(
            DecisionType::REPLAN,
            "需要初始路径规划");
    }
    
    // 默认继续跟随当前路径
    return DecisionResult(
        DecisionType::FOLLOW_PATH,
        "继续跟随路径");
}

void DecisionMaker::setSafetyDistance(double distance) {
    safety_distance_ = distance;
}

void DecisionMaker::setEmergencyDistance(double distance) {
    emergency_distance_ = distance;
}

std::pair<bool, double> DecisionMaker::checkObstaclesAhead(
    const std::vector<auto_perception::DetectedObject>& objects,
    const geometry_msgs::msg::PoseStamped& current_pose,
    double heading_angle,
    double max_distance,
    double corridor_width) {
    
    bool has_obstacle = false;
    double min_distance = max_distance;
    
    for (const auto& obj : objects) {
        // 计算障碍物相对于车辆的位置
        double dx = obj.pose.position.x - current_pose.pose.position.x;
        double dy = obj.pose.position.y - current_pose.pose.position.y;
        
        // 转换到车辆坐标系中
        double local_x = dx * cos(-heading_angle) - dy * sin(-heading_angle);
        double local_y = dx * sin(-heading_angle) + dy * cos(-heading_angle);
        
        // 判断障碍物是否在前方
        if (local_x > 0 && local_x < max_distance) {
            // 判断障碍物是否在车道宽度范围内
            if (std::abs(local_y) < corridor_width / 2.0) {
                has_obstacle = true;
                double distance = std::hypot(local_x, local_y) - obj.length / 2.0;
                min_distance = std::min(min_distance, distance);
            }
        }
    }
    
    return {has_obstacle, min_distance};
}

} // namespace auto_planning

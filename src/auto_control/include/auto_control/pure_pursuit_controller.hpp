#ifndef PURE_PURSUIT_CONTROLLER_HPP_
#define PURE_PURSUIT_CONTROLLER_HPP_

#include <vector>
#include <memory>
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/path_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace auto_control {

struct ControlCommand {
    double steering_angle;  // 转向角度（弧度）
    double throttle;        // 油门（范围：0-1）
    double brake;           // 制动（范围：0-1）
    
    ControlCommand(double steering = 0.0, double throttle = 0.0, double brake = 0.0)
        : steering_angle(steering), throttle(throttle), brake(brake) {}
};

class PurePursuitController {
public:
    PurePursuitController();
    virtual ~PurePursuitController() = default;

    // 设置路径
    void setPath(const auto_msgs::msg::PlanningPath& path);
    
    // 计算控制命令
    ControlCommand calculateControl(
        const geometry_msgs::msg::Pose& current_pose,
        const geometry_msgs::msg::Twist& current_velocity);
    
    // 设置预瞄距离（米）
    void setLookaheadDistance(double distance);
    
    // 设置最大速度（米/秒）
    void setMaxVelocity(double velocity);
    
    // 检查是否有路径
    bool hasPath() const;
    
    // 检查是否到达目标
    bool reachedGoal() const;

private:
    // 查找最近的路径点索引
    size_t findClosestPointIndex(const geometry_msgs::msg::Pose& pose);
    
    // 查找预瞄点
    std::pair<bool, geometry_msgs::msg::Pose> findLookaheadPoint(
        const geometry_msgs::msg::Pose& pose, size_t start_index);
    
    // 计算两点之间的欧氏距离
    double calculateDistance(
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2);
    
    // 路径点
    std::vector<auto_msgs::msg::PathPoint> path_points_;
    
    // 最后处理的索引
    size_t last_closest_index_;
    
    // 预瞄距离（米）
    double lookahead_distance_;
    
    // 车辆轴距（米）
    double wheelbase_;
    
    // 最大速度（米/秒）
    double max_velocity_;
    
    // 目标速度控制参数
    double speed_p_gain_;
    
    // 是否已经设置路径
    bool has_path_;
    
    // 是否到达目标
    bool reached_goal_;
};

} // namespace auto_control

#endif // PURE_PURSUIT_CONTROLLER_HPP_

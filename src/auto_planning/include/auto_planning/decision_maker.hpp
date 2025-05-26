#ifndef DECISION_MAKER_HPP_
#define DECISION_MAKER_HPP_

#include <vector>
#include <memory>
#include <string>
#include "auto_msgs/msg/planning_request.hpp"
#include "auto_perception/object_detector.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace auto_planning {

// 决策结果类型
enum class DecisionType {
    FOLLOW_PATH,      // 跟随当前路径
    REPLAN,           // 需要重新规划路径
    STOP,             // 停车
    EMERGENCY_STOP    // 紧急停车
};

struct DecisionResult {
    DecisionType type;
    std::string reason;
    double distance_to_obstacle;  // 如果有障碍物，到最近障碍物的距离
    
    DecisionResult(
        DecisionType type_ = DecisionType::FOLLOW_PATH,
        const std::string& reason_ = "",
        double distance_to_obstacle_ = -1.0)
        : type(type_), reason(reason_), distance_to_obstacle(distance_to_obstacle_) {}
};

class DecisionMaker {
public:
    DecisionMaker();
    virtual ~DecisionMaker() = default;

    // 基于感知结果和当前状态做出决策
    DecisionResult makeDecision(
        const std::vector<auto_perception::DetectedObject>& detected_objects,
        const geometry_msgs::msg::PoseStamped& current_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        bool has_path,
        double current_speed);
    
    // 设置安全距离（米）
    void setSafetyDistance(double distance);
    
    // 设置紧急停车距离（米）
    void setEmergencyDistance(double distance);

private:
    // 检查前方是否有障碍物
    std::pair<bool, double> checkObstaclesAhead(
        const std::vector<auto_perception::DetectedObject>& objects,
        const geometry_msgs::msg::PoseStamped& current_pose,
        double heading_angle,
        double max_distance,
        double corridor_width);
        
    // 安全距离（米）
    double safety_distance_;
    
    // 紧急停车距离（米）
    double emergency_distance_;
};

} // namespace auto_planning

#endif // DECISION_MAKER_HPP_

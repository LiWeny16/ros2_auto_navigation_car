#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "auto_perception/object_detector.hpp"

#include "auto_planning/a_star_planner.hpp"
#include "auto_planning/optimized_a_star_planner.hpp"
#include "auto_planning/hybrid_a_star_planner.hpp"
#include "auto_planning/decision_maker.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace auto_planning {

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner_node") {
    // 创建规划器
    a_star_planner_ = std::make_unique<AStarPlanner>();
    optimized_a_star_planner_ = std::make_unique<OptimizedAStarPlanner>();
    hybrid_a_star_planner_ = std::make_unique<HybridAStarPlanner>();
    decision_maker_ = std::make_unique<DecisionMaker>();
        
        // 创建订阅者
        map_sub_ = this->create_subscription<auto_msgs::msg::GridMap>(
            "grid_map", 10, std::bind(&PathPlannerNode::mapCallback, this, _1));
        
        request_sub_ = this->create_subscription<auto_msgs::msg::PlanningRequest>(
            "planning_request", 10, std::bind(&PathPlannerNode::planningRequestCallback, this, _1));
            
        objects_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "detected_objects_markers", 10, std::bind(&PathPlannerNode::objectsCallback, this, _1));
        
        // 创建发布者
        path_pub_ = this->create_publisher<auto_msgs::msg::PlanningPath>("planning_path", 10);
        
        // 创建定时器以执行持续决策
        decision_timer_ = this->create_wall_timer(
            200ms, std::bind(&PathPlannerNode::decisionLoop, this));
        
        // 设置决策参数
        decision_maker_->setSafetyDistance(5.0);
        decision_maker_->setEmergencyDistance(2.0);
        
        RCLCPP_INFO(this->get_logger(), "路径规划节点已启动");
    }

private:
    void mapCallback(const auto_msgs::msg::GridMap::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到地图，尺寸: %dx%d", msg->width, msg->height);
        current_map_ = *msg;
    }
    
    void planningRequestCallback(const auto_msgs::msg::PlanningRequest::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到规划请求，类型: %s", msg->planner_type.c_str());
        
        if (!haveValidMap()) {
            RCLCPP_ERROR(this->get_logger(), "尚未收到有效地图，无法规划");
            return;
        }
        
        // 保存当前目标
        current_goal_ = msg->goal;
        have_goal_ = true;
        
        auto_msgs::msg::PlanningPath path;
        
        // 根据请求选择不同的规划器
        if (msg->planner_type == "astar") {
            RCLCPP_INFO(this->get_logger(), "使用A*规划路径");
            path = a_star_planner_->plan(current_map_, msg->start, msg->goal);
        } else if (msg->planner_type == "optimized_astar") {
            RCLCPP_INFO(this->get_logger(), "使用优化A*规划路径");
            
            // 解析配置参数（如果有）
            OptimizedAStarPlanner::AStarConfig config;
            std::string frame_id = msg->header.frame_id;
            
            if (frame_id.find("config:") == 0) {
                // 解析配置字符串
                std::string config_str = frame_id.substr(7);  // 跳过"config:"
                
                auto parse_bool_param = [&config_str](const std::string& param_name, bool& value) {
                    std::string pattern = param_name + "=";
                    size_t pos = config_str.find(pattern);
                    if (pos != std::string::npos) {
                        value = (config_str[pos + pattern.size()] == '1');
                    }
                };
                
                auto parse_int_param = [&config_str](const std::string& param_name, int& value) {
                    std::string pattern = param_name + "=";
                    size_t pos = config_str.find(pattern);
                    if (pos != std::string::npos) {
                        size_t end_pos = config_str.find(";", pos);
                        if (end_pos != std::string::npos) {
                            std::string val_str = config_str.substr(pos + pattern.size(), end_pos - pos - pattern.size());
                            value = std::stoi(val_str);
                        }
                    }
                };
                
                auto parse_double_param = [&config_str](const std::string& param_name, double& value) {
                    std::string pattern = param_name + "=";
                    size_t pos = config_str.find(pattern);
                    if (pos != std::string::npos) {
                        size_t end_pos = config_str.find(";", pos);
                        if (end_pos != std::string::npos) {
                            std::string val_str = config_str.substr(pos + pattern.size(), end_pos - pos - pattern.size());
                            value = std::stod(val_str);
                        }
                    }
                };
                
                parse_bool_param("parallel", config.enable_parallel_search);
                parse_bool_param("heuristic", config.use_improved_heuristic);
                parse_int_param("iterations", config.max_iterations);
                parse_double_param("tolerance", config.goal_tolerance);
                parse_int_param("resolution", config.grid_resolution);
                parse_bool_param("recycling", config.use_node_recycling);
                parse_double_param("timeout", config.planning_timeout);
            }
            
            path = optimized_a_star_planner_->plan(current_map_, msg->start, msg->goal, config);
        } else if (msg->planner_type == "hybrid_astar") {
            RCLCPP_INFO(this->get_logger(), "使用Hybrid A*规划路径");
            path = hybrid_a_star_planner_->plan(current_map_, msg->start, msg->goal);
        } else {
            RCLCPP_ERROR(this->get_logger(), "未知规划器类型: %s", msg->planner_type.c_str());
            return;
        }
        
        if (path.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "未能找到可行路径");
            have_path_ = false;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "规划完成，路径长度: %.2f米，耗时: %.2f秒", 
                   path.total_distance, path.planning_time);
        
        // 设置路径状态
        have_path_ = true;
        
        // 发布规划路径
        path_pub_->publish(path);
    }
    
    void objectsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        // 将标记数组转换为检测到的对象列表
        detected_objects_.clear();
        
        for (const auto& marker : msg->markers) {
            // 只处理类型为CUBE的标记，这些是物体标记
            if (marker.type == visualization_msgs::msg::Marker::CUBE && 
                marker.ns == "detected_objects") {
                
                auto_perception::DetectedObject obj(
                    marker.id,
                    marker.pose,
                    marker.scale.y,  // width
                    marker.scale.x,  // length
                    marker.scale.z   // height
                );
                
                // 根据颜色确定对象类型
                if (marker.color.r > 0.8 && marker.color.g < 0.2 && marker.color.b < 0.2) {
                    obj.classification = "vehicle";
                } else if (marker.color.g > 0.8 && marker.color.r < 0.2 && marker.color.b < 0.2) {
                    obj.classification = "pedestrian";
                } else if (marker.color.b > 0.8 && marker.color.r < 0.2 && marker.color.g < 0.2) {
                    obj.classification = "bicycle";
                } else {
                    obj.classification = "obstacle";
                }
                
                obj.confidence = marker.color.a / 0.8;  // 将透明度转换回置信度
                
                detected_objects_.push_back(obj);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "接收到 %zu 个检测到的对象", detected_objects_.size());
    }
    
    void decisionLoop() {
        // 如果没有当前目标，无法做决策
        if (!have_goal_) {
            return;
        }
        
        // 获取最新的车辆位置（在实际系统中应该从其他模块获取）
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.stamp = this->now();
        current_pose.header.frame_id = "map";
        // 简化示例，使用固定位置
        current_pose.pose.position.x = 10.0;
        current_pose.pose.position.y = 10.0;
        current_pose.pose.orientation.w = 1.0;
        
        // 调用决策器
        auto decision = decision_maker_->makeDecision(
            detected_objects_,
            current_pose,
            current_goal_,
            have_path_,
            2.0  // 假设当前速度为2.0 m/s
        );
        
        // 根据决策结果执行操作
        switch (decision.type) {
            case DecisionType::REPLAN:
                RCLCPP_INFO(this->get_logger(), "决策：重新规划，原因: %s", decision.reason.c_str());
                // 触发重新规划
                auto_msgs::msg::PlanningRequest request;
                request.header.stamp = this->now();
                request.header.frame_id = "map";
                request.start = current_pose;
                request.goal = current_goal_;
                request.planner_type = "optimized_astar";  // 使用优化版A*重新规划
                request.consider_kinematic = true;
                planningRequestCallback(std::make_shared<auto_msgs::msg::PlanningRequest>(request));
                break;
                
            case DecisionType::STOP:
                RCLCPP_INFO(this->get_logger(), "决策：停车，原因: %s", decision.reason.c_str());
                // 在实际系统中，应该发送停车命令
                break;
                
            case DecisionType::EMERGENCY_STOP:
                RCLCPP_INFO(this->get_logger(), "决策：紧急停车，原因: %s", decision.reason.c_str());
                // 在实际系统中，应该发送紧急停车命令
                break;
                
            case DecisionType::FOLLOW_PATH:
                // 继续跟随当前路径，不需要特殊处理
                break;
        }
    }
    
    bool haveValidMap() {
        return current_map_.width > 0 && current_map_.height > 0 && !current_map_.data.empty();
    }
    
    // 规划器
    std::unique_ptr<AStarPlanner> a_star_planner_;
    std::unique_ptr<OptimizedAStarPlanner> optimized_a_star_planner_;
    std::unique_ptr<HybridAStarPlanner> hybrid_a_star_planner_;
    std::unique_ptr<DecisionMaker> decision_maker_;
    
    // 当前地图
    auto_msgs::msg::GridMap current_map_;
    
    // 检测到的对象
    std::vector<auto_perception::DetectedObject> detected_objects_;
    
    // 当前目标和路径状态
    geometry_msgs::msg::PoseStamped current_goal_;
    bool have_goal_ = false;
    bool have_path_ = false;
    
    // 订阅者
    rclcpp::Subscription<auto_msgs::msg::GridMap>::SharedPtr map_sub_;
    rclcpp::Subscription<auto_msgs::msg::PlanningRequest>::SharedPtr request_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr objects_sub_;
    
    // 发布者
    rclcpp::Publisher<auto_msgs::msg::PlanningPath>::SharedPtr path_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr decision_timer_;
};

} // namespace auto_planning

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<auto_planning::PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

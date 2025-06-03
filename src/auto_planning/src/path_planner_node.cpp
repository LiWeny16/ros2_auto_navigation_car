#include <memory>
#include <string>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/empty.hpp"
// #include "auto_perception/object_detector.hpp"

#include "auto_planning/a_star_planner.hpp"
#include "auto_planning/optimized_a_star_planner.hpp"
#include "auto_planning/hybrid_a_star_planner.hpp"
// #include "auto_planning/decision_maker.hpp"

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
    // decision_maker_ = std::make_unique<DecisionMaker>();
        
        // 创建订阅者
        map_sub_ = this->create_subscription<auto_msgs::msg::GridMap>(
            "grid_map", 10, std::bind(&PathPlannerNode::mapCallback, this, _1));
        
        request_sub_ = this->create_subscription<auto_msgs::msg::PlanningRequest>(
            "planning_request", 10, std::bind(&PathPlannerNode::planningRequestCallback, this, _1));
            
        objects_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "detected_objects_markers", 10, std::bind(&PathPlannerNode::objectsCallback, this, _1));
        
        // 订阅路径清理消息
        clear_path_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "clear_path", 10, std::bind(&PathPlannerNode::clearPathCallback, this, _1));
        
        // 创建发布者
        path_pub_ = this->create_publisher<auto_msgs::msg::PlanningPath>("planning_path", 10);
        path_clear_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clear_path_visualization", 10);
        
        // 创建一个单次触发的定时器，用于异步重新规划
        replan_timer_ = this->create_wall_timer(
            10ms, std::bind(&PathPlannerNode::executeReplan, this));
        replan_timer_->cancel(); // 初始时取消，只在需要时触发
        
        // 获取参数
        this->declare_parameter("map_change_threshold", 0.05);  // 地图变化阈值
        this->declare_parameter("replan_delay_ms", 100);        // 重新规划延迟
        this->declare_parameter("max_map_age_ms", 15000);       // 最大地图年龄
        
        map_change_threshold_ = this->get_parameter("map_change_threshold").as_double();
        replan_delay_ms_ = this->get_parameter("replan_delay_ms").as_int();
        max_map_age_ms_ = this->get_parameter("max_map_age_ms").as_int();
        
        RCLCPP_INFO(this->get_logger(), "路径规划节点已启动 - 支持实时地图更新响应");
        RCLCPP_INFO(this->get_logger(), "参数配置: 地图变化阈值=%.3f, 重规划延迟=%dms, 最大地图年龄=%dms", 
                   map_change_threshold_, replan_delay_ms_, max_map_age_ms_);
    }

private:
    void mapCallback(const auto_msgs::msg::GridMap::SharedPtr msg) {
        auto callback_start = std::chrono::steady_clock::now();
        
        RCLCPP_INFO(this->get_logger(), "收到地图，尺寸: %dx%d, 时间戳: %d.%d", 
                   msg->width, msg->height, msg->header.stamp.sec, msg->header.stamp.nanosec);
        
        // 检查地图是否发生了显著变化
        bool map_changed = hasMapChanged(*msg);
        bool is_first_map = !have_valid_map_;
        
        // 更新当前地图
        current_map_ = *msg;
        have_valid_map_ = true;
        last_map_timestamp_ = this->now();
        
        if (is_first_map) {
            RCLCPP_INFO(this->get_logger(), "收到首个地图");
        } else if (map_changed) {
            RCLCPP_INFO(this->get_logger(), "检测到地图变化 (变化率: %.3f)", last_map_change_ratio_);
            
            // 如果有当前目标，立即触发重新规划
            if (have_goal_) {
                RCLCPP_INFO(this->get_logger(), "地图变化 -> 触发重新规划");
                triggerReplan("map_changed");
            }
        } else {
            RCLCPP_DEBUG(this->get_logger(), "地图无显著变化 (变化率: %.3f < %.3f)", 
                        last_map_change_ratio_, map_change_threshold_);
        }
        
        // 记录回调处理时间
        auto callback_end = std::chrono::steady_clock::now();
        auto callback_duration = std::chrono::duration<double, std::milli>(callback_end - callback_start);
        RCLCPP_DEBUG(this->get_logger(), "地图回调处理耗时: %.2fms", callback_duration.count());
    }
    
    bool hasMapChanged(const auto_msgs::msg::GridMap& new_map) {
        // 如果是第一次收到地图
        if (!have_valid_map_ || current_map_.data.empty()) {
            last_map_change_ratio_ = 1.0;
            return true;
        }
        
        // 检查地图尺寸是否变化
        if (new_map.width != current_map_.width || 
            new_map.height != current_map_.height ||
            std::abs(new_map.resolution - current_map_.resolution) > 1e-6) {
            RCLCPP_INFO(this->get_logger(), "地图尺寸或分辨率发生变化");
            last_map_change_ratio_ = 1.0;
            return true;
        }
        
        // 检查地图数据变化
        if (new_map.data.size() != current_map_.data.size()) {
            last_map_change_ratio_ = 1.0;
            return true;
        }
        
        // 计算变化的像素比例
        size_t changed_cells = 0;
        for (size_t i = 0; i < new_map.data.size(); ++i) {
            if (new_map.data[i] != current_map_.data[i]) {
                changed_cells++;
            }
        }
        
        last_map_change_ratio_ = static_cast<double>(changed_cells) / new_map.data.size();
        
        return last_map_change_ratio_ > map_change_threshold_;
    }
    
    void triggerReplan(const std::string& reason) {
        if (pending_replan_) {
            RCLCPP_DEBUG(this->get_logger(), "已有待处理的重新规划，跳过触发");
            return;
        }
        
        pending_replan_ = true;
        replan_reason_ = reason;
        
        // 重新设置定时器，延迟执行重新规划
        replan_timer_->cancel();
        replan_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(replan_delay_ms_), 
            std::bind(&PathPlannerNode::executeReplan, this));
        
        RCLCPP_DEBUG(this->get_logger(), "已触发重新规划 (原因: %s), 将在%dms后执行", 
                    reason.c_str(), replan_delay_ms_);
    }
    
    void executeReplan() {
        replan_timer_->cancel(); // 取消定时器，确保只执行一次
        
        if (!pending_replan_) {
            return;
        }
        
        pending_replan_ = false;
        
        RCLCPP_INFO(this->get_logger(), "执行重新规划 (原因: %s)", replan_reason_.c_str());
        
        if (!haveValidMap()) {
            RCLCPP_ERROR(this->get_logger(), "地图无效，无法重新规划");
            return;
        }
        
        if (!have_goal_) {
            RCLCPP_WARN(this->get_logger(), "没有目标，无法重新规划");
            return;
        }
        
        // 检查地图年龄
        auto map_age = this->now() - last_map_timestamp_;
        if (map_age.nanoseconds() / 1e6 > max_map_age_ms_) {
            RCLCPP_WARN(this->get_logger(), "地图数据过旧 (%.1fs)，等待新地图", 
                       map_age.nanoseconds() / 1e9);
            return;
        }
        
        // 执行路径规划
        auto path = executePlanning(current_goal_, current_planner_type_);
        
        if (path.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "重新规划失败，未找到可行路径");
            have_path_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "重新规划成功，路径长度: %.2f米，耗时: %.2f秒", 
                       path.total_distance, path.planning_time);
            have_path_ = true;
            path_pub_->publish(path);
        }
    }
    
    void planningRequestCallback(const auto_msgs::msg::PlanningRequest::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到规划请求，类型: %s", msg->planner_type.c_str());
        
        if (!haveValidMap()) {
            RCLCPP_ERROR(this->get_logger(), "尚未收到有效地图，无法规划");
            return;
        }
        
        // 检查地图时间戳
        auto map_age = this->now() - last_map_timestamp_;
        if (map_age.nanoseconds() / 1e6 > max_map_age_ms_) {
            RCLCPP_WARN(this->get_logger(), "地图数据过旧 (%.1fs)，建议等待新地图", 
                       map_age.nanoseconds() / 1e9);
        }
        
        // 保存当前目标和规划器类型
        current_goal_ = msg->goal;
        current_start_ = msg->start;
        current_planner_type_ = msg->planner_type;
        have_goal_ = true;
        
        // 立即执行规划
        auto path = executePlanning(msg->goal, msg->planner_type, &msg->start);
        
        if (path.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "未能找到可行路径");
            have_path_ = false;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "规划完成，路径长度: %.2f米，耗时: %.2f秒", 
                   path.total_distance, path.planning_time);
        
        have_path_ = true;
        path_pub_->publish(path);
    }
    
    auto_msgs::msg::PlanningPath executePlanning(
        const geometry_msgs::msg::PoseStamped& goal,
        const std::string& planner_type,
        const geometry_msgs::msg::PoseStamped* start_override = nullptr) {
        
        auto planning_start = std::chrono::steady_clock::now();
        
        // 确定起点
        geometry_msgs::msg::PoseStamped start_pose;
        if (start_override) {
            start_pose = *start_override;
        } else if (have_goal_) {
            start_pose = current_start_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "没有起点信息");
            return auto_msgs::msg::PlanningPath();
        }
        
        auto_msgs::msg::PlanningPath path;
        
        try {
            // 根据规划器类型选择不同的规划算法
            if (planner_type == "astar") {
                RCLCPP_DEBUG(this->get_logger(), "使用A*规划路径");
                path = a_star_planner_->plan(current_map_, start_pose, goal);
            } else if (planner_type == "optimized_astar") {
                RCLCPP_DEBUG(this->get_logger(), "使用优化A*规划路径");
                
                // 使用默认配置
                AStarConfig config;
                path = optimized_a_star_planner_->plan(current_map_, start_pose, goal, config);
            } else if (planner_type == "hybrid_astar") {
                RCLCPP_DEBUG(this->get_logger(), "使用Hybrid A*规划路径");
                path = hybrid_a_star_planner_->plan(current_map_, start_pose, goal);
            } else {
                RCLCPP_ERROR(this->get_logger(), "未知规划器类型: %s", planner_type.c_str());
                return auto_msgs::msg::PlanningPath();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "规划过程中发生异常: %s", e.what());
            return auto_msgs::msg::PlanningPath();
        }
        
        // 记录规划总时间（包括前面的处理时间）
        auto planning_end = std::chrono::steady_clock::now();
        auto total_duration = std::chrono::duration<double>(planning_end - planning_start);
        
        if (!path.points.empty()) {
            // 更新路径的总处理时间
            path.planning_time = total_duration.count();
            path.header.stamp = this->now();
        }
        
        return path;
    }
    
    void objectsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        // 处理检测到的对象，如果对象发生变化也可以触发重新规划
        RCLCPP_DEBUG(this->get_logger(), "接收到 %zu 个可视化标记", msg->markers.size());
        
        // 可以在这里添加对象变化检测逻辑
        // 如果检测到新的障碍物，也可以触发重新规划
    }
    
    void clearPathCallback(const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到路径清理请求");
        
        if (have_path_) {
            RCLCPP_INFO(this->get_logger(), "清理当前路径状态");
            have_path_ = false;
            // 不要发布空路径，这会导致路径消失
            // path_pub_->publish(auto_msgs::msg::PlanningPath());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "没有路径需要清理");
        }
    }
    
    bool haveValidMap() {
        return have_valid_map_ && 
               current_map_.width > 0 && 
               current_map_.height > 0 && 
               !current_map_.data.empty();
    }

    // 规划器
    std::unique_ptr<AStarPlanner> a_star_planner_;
    std::unique_ptr<OptimizedAStarPlanner> optimized_a_star_planner_;
    std::unique_ptr<HybridAStarPlanner> hybrid_a_star_planner_;
    // std::unique_ptr<DecisionMaker> decision_maker_;
    
    // 地图状态
    auto_msgs::msg::GridMap current_map_;
    bool have_valid_map_ = false;
    rclcpp::Time last_map_timestamp_;
    double last_map_change_ratio_ = 0.0;
    
    // 规划状态
    geometry_msgs::msg::PoseStamped current_goal_;
    geometry_msgs::msg::PoseStamped current_start_;
    std::string current_planner_type_ = "hybrid_astar";
    bool have_goal_ = false;
    bool have_path_ = false;
    
    // 重新规划控制
    bool pending_replan_ = false;
    std::string replan_reason_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    
    // 参数
    double map_change_threshold_;
    int replan_delay_ms_;
    int max_map_age_ms_;
    
    // 订阅者和发布者
    rclcpp::Subscription<auto_msgs::msg::GridMap>::SharedPtr map_sub_;
    rclcpp::Subscription<auto_msgs::msg::PlanningRequest>::SharedPtr request_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr objects_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_path_sub_;
    rclcpp::Publisher<auto_msgs::msg::PlanningPath>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_clear_vis_pub_;
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

#ifndef AUTO_SIMULATION_SIMULATION_NODE_HPP
#define AUTO_SIMULATION_SIMULATION_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

#include "auto_simulation/map_generator.hpp"
#include "auto_simulation/visualization_manager.hpp"
#include "auto_simulation/planning_request_manager.hpp"

namespace auto_simulation {

/**
 * @brief 自动驾驶仿真节点，协调地图生成、路径规划和可视化
 */
class SimulationNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    SimulationNode();

private:
    /**
     * @brief 发布地图并请求规划路径
     */
    void publishMapAndRequestPlanning();
    
    /**
     * @brief 发送延迟规划请求
     */
    void sendDelayedPlanningRequest();
    
    /**
     * @brief 安排延迟规划请求
     */
    void scheduleDelayedPlanningRequest();
    
    /**
     * @brief 路径规划回调函数
     * 
     * @param msg 规划路径消息
     */
    void pathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg);

    // 模块组件
    std::unique_ptr<MapGenerator> map_generator_;                     // 地图生成器
    std::unique_ptr<VisualizationManager> visualization_manager_;     // 可视化管理器
    std::unique_ptr<PlanningRequestManager> planning_request_manager_; // 规划请求管理器

    // 通信组件
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_clear_pub_; // 路径清除发布者
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_; // 路径订阅者

    // 定时器
    rclcpp::TimerBase::SharedPtr map_timer_;                 // 地图定时器
    rclcpp::TimerBase::SharedPtr planning_delay_timer_;      // 规划延迟定时器

    // 参数
    double map_update_interval_;                             // 地图更新间隔
};

} // namespace auto_simulation

#endif // AUTO_SIMULATION_SIMULATION_NODE_HPP

#ifndef AUTO_SIMULATION_PLANNING_REQUEST_MANAGER_HPP
#define AUTO_SIMULATION_PLANNING_REQUEST_MANAGER_HPP

#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "auto_simulation/json_utils.hpp"

namespace auto_simulation {

/**
 * @brief 负责生成和管理规划请求的类
 */
class PlanningRequestManager {
public:
    /**
     * @brief 构造函数
     * 
     * @param node ROS2节点指针
     */
    explicit PlanningRequestManager(rclcpp::Node* node);

    /**
     * @brief 发送规划请求
     * 
     * @param map 当前地图，用于在地图上找到合适的起点和终点
     * @param map_sequence 地图序列号，用于确保可重现性
     * @return 是否成功发送请求
     */
    bool sendPlanningRequest(const auto_msgs::msg::GridMap& map, int map_sequence);

    /**
     * @brief 检查是否有有效的起点和终点
     * 
     * @return 是否有有效起点和终点
     */
    bool hasValidStartGoal() const;

    /**
     * @brief 获取当前起点
     * 
     * @return 当前起点
     */
    const geometry_msgs::msg::PoseStamped& getCurrentStart() const;

    /**
     * @brief 获取当前终点
     * 
     * @return 当前终点
     */
    const geometry_msgs::msg::PoseStamped& getCurrentGoal() const;

private:
    /**
     * @brief 在地图上找到有效的起点和终点
     * 
     * @param map 地图
     * @param gen 随机数生成器
     * @param request 规划请求（将被填充）
     * @return 是否成功找到
     */
    bool findValidStartGoal(const auto_msgs::msg::GridMap& map, 
                             std::mt19937& gen, 
                             auto_msgs::msg::PlanningRequest& request);

    rclcpp::Node* node_;                                                              // 所属ROS2节点
    rclcpp::Publisher<auto_msgs::msg::PlanningRequest>::SharedPtr planning_request_pub_;   // 规划请求发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_request_json_pub_;       // JSON格式规划请求发布者
    bool have_valid_start_goal_;                                                      // 是否有有效起点和终点
    geometry_msgs::msg::PoseStamped current_start_;                                   // 当前起点
    geometry_msgs::msg::PoseStamped current_goal_;                                    // 当前终点
};

} // namespace auto_simulation

#endif // AUTO_SIMULATION_PLANNING_REQUEST_MANAGER_HPP

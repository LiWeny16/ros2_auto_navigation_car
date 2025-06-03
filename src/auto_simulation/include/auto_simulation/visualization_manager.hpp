#ifndef AUTO_SIMULATION_VISUALIZATION_MANAGER_HPP
#define AUTO_SIMULATION_VISUALIZATION_MANAGER_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auto_msgs/msg/planning_path.hpp"

namespace auto_simulation {

/**
 * @brief 负责可视化地图、路径和车辆的类
 */
class VisualizationManager {
public:
    /**
     * @brief 构造函数
     * 
     * @param node ROS2节点指针
     */
    explicit VisualizationManager(rclcpp::Node* node);

    /**
     * @brief 可视化地图
     * 
     * @param map 要可视化的地图
     */
    void visualizeMap(const auto_msgs::msg::GridMap& map);

    /**
     * @brief 可视化起点和终点
     * 
     * @param start 起点
     * @param goal 终点
     */
    void visualizeStartGoal(const geometry_msgs::msg::PoseStamped& start, 
                           const geometry_msgs::msg::PoseStamped& goal);
    
    /**
     * @brief 可视化规划路径
     * 
     * @param path 规划路径
     */
    void visualizePath(const auto_msgs::msg::PlanningPath& path);
    
    /**
     * @brief 清除旧路径可视化
     */
    void clearOldPath();

private:
    rclcpp::Node* node_;                                                             // 所属ROS2节点
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;     // 可视化标记发布者
};

} // namespace auto_simulation

#endif // AUTO_SIMULATION_VISUALIZATION_MANAGER_HPP

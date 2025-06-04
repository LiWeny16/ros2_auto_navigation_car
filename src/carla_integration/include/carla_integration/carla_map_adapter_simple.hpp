#ifndef CARLA_MAP_ADAPTER_SIMPLE_HPP_
#define CARLA_MAP_ADAPTER_SIMPLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include "auto_msgs/msg/grid_map.hpp"

namespace carla_integration {

/**
 * @brief CARLA 地图适配器 (简化版)
 * 
 * 该类负责将标准的占用栅格地图转换为项目使用的 GridMap 格式
 * 暂时不直接依赖 CARLA 消息，而是使用标准 ROS 消息
 */
class CarlaMapAdapter : public rclcpp::Node {
public:
    CarlaMapAdapter();

private:
    /**
     * @brief 占用栅格地图回调函数
     * @param msg 占用栅格地图消息
     */
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief 将占用栅格转换为 GridMap
     * @param occupancy_grid 占用栅格
     * @return 转换后的 GridMap
     */
    auto_msgs::msg::GridMap convertToGridMap(const nav_msgs::msg::OccupancyGrid& occupancy_grid);
    
    /**
     * @brief 定时发布地图信息
     */
    void publishMapInfo();

    // ROS 2 订阅者
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    
    // ROS 2 发布者
    rclcpp::Publisher<auto_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    
    // 定时器
    rclcpp::TimerInterface::SharedPtr map_timer_;
    
    // 缓存的地图数据
    auto_msgs::msg::GridMap current_grid_map_;
    bool map_initialized_;
    
    // 参数
    double map_resolution_;
    double map_width_;
    double map_height_;
    std::string map_frame_;
};

} // namespace carla_integration

#endif // CARLA_MAP_ADAPTER_SIMPLE_HPP_

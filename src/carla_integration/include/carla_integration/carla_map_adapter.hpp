#ifndef CARLA_MAP_ADAPTER_HPP_
#define CARLA_MAP_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include "auto_msgs/msg/grid_map.hpp"

// CARLA ROS Bridge messages (commented out for now)
// #include "carla_msgs/msg/carla_world_info.hpp"

namespace carla_integration {

/**
 * @brief CARLA 地图适配器
 * 
 * 该类负责将 CARLA 仿真器中的地图数据转换为项目使用的 GridMap 格式
 * 主要功能：
 * 1. 订阅 CARLA 地图信息
 * 2. 转换为 auto_msgs::msg::GridMap 格式
 * 3. 发布给路径规划模块使用
 */
class CarlaMapAdapter : public rclcpp::Node {
public:
    CarlaMapAdapter();

private:
    /**
     * @brief 世界信息回调函数
     * @param msg 世界信息消息
     */
    void worldInfoCallback(const std_msgs::msg::String::SharedPtr msg);
    
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr world_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    
    // ROS 2 发布者
    rclcpp::Publisher<auto_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr map_timer_;
    
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

#endif // CARLA_MAP_ADAPTER_HPP_

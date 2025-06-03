#ifndef AUTO_SIMULATION_MAP_GENERATOR_HPP
#define AUTO_SIMULATION_MAP_GENERATOR_HPP

#include <memory>
#include <random>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "std_msgs/msg/string.hpp"
#include "auto_simulation/json_utils.hpp"

namespace auto_simulation {

/**
 * @brief 负责生成和管理模拟地图的类
 */
class MapGenerator {
public:
    /**
     * @brief 构造函数
     * 
     * @param node ROS2节点指针
     */
    explicit MapGenerator(rclcpp::Node* node);

    /**
     * @brief 生成并发布一个新的地图
     * 
     * @return 生成的地图
     */
    auto_msgs::msg::GridMap generateAndPublishMap();

    /**
     * @brief 获取当前地图
     * 
     * @return 当前地图的常量引用
     */
    const auto_msgs::msg::GridMap& getCurrentMap() const;

    /**
     * @brief 获取地图序列号
     * 
     * @return 当前地图序列号
     */
    int getMapSequence() const;

private:
    /**
     * @brief 创建并初始化地图
     * 
     * @return 初始化后的地图
     */
    auto_msgs::msg::GridMap createMap();

    /**
     * @brief 添加随机障碍物到地图
     * 
     * @param map 待修改的地图
     * @param gen 随机数生成器
     */
    void addRandomObstacles(auto_msgs::msg::GridMap& map, std::mt19937& gen);

    /**
     * @brief 添加线性障碍物（墙壁）到地图
     * 
     * @param map 待修改的地图
     * @param gen 随机数生成器
     */
    void addLinearObstacles(auto_msgs::msg::GridMap& map, std::mt19937& gen);

    rclcpp::Node* node_;                                             // 所属ROS2节点
    rclcpp::Publisher<auto_msgs::msg::GridMap>::SharedPtr map_pub_;  // 地图发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_json_pub_;  // JSON格式地图发布者
    auto_msgs::msg::GridMap current_map_;                           // 当前地图
    int map_sequence_;                                              // 地图序列号
};

} // namespace auto_simulation

#endif // AUTO_SIMULATION_MAP_GENERATOR_HPP

#include "carla_integration/carla_map_adapter.hpp"
#include <rclcpp/rclcpp.hpp>

namespace carla_integration {

CarlaMapAdapter::CarlaMapAdapter() 
    : Node("carla_map_adapter"), map_initialized_(false) {
    
    // 声明参数
    this->declare_parameter("map_resolution", 0.1);
    this->declare_parameter("map_width", 200.0);
    this->declare_parameter("map_height", 200.0);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("publish_rate", 1.0);
    
    // 获取参数
    map_resolution_ = this->get_parameter("map_resolution").as_double();
    map_width_ = this->get_parameter("map_width").as_double();
    map_height_ = this->get_parameter("map_height").as_double();
    map_frame_ = this->get_parameter("map_frame").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    
    RCLCPP_INFO(this->get_logger(), "CARLA Map Adapter initialized");
    RCLCPP_INFO(this->get_logger(), "Map resolution: %.2f m", map_resolution_);
    RCLCPP_INFO(this->get_logger(), "Map size: %.1fx%.1f m", map_width_, map_height_);
    
    // 创建订阅者
    world_info_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/world_info", 10,
        std::bind(&CarlaMapAdapter::worldInfoCallback, this, std::placeholders::_1));
    
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&CarlaMapAdapter::occupancyGridCallback, this, std::placeholders::_1));
    
    // 创建发布者
    grid_map_pub_ = this->create_publisher<auto_msgs::msg::GridMap>(
        "/map/grid_map", 10);
    
    // 创建定时器，定期发布地图信息
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    map_timer_ = this->create_wall_timer(
        timer_period, std::bind(&CarlaMapAdapter::publishMapInfo, this));
}

void CarlaMapAdapter::worldInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received world info: %s", msg->data.c_str());
    
    // 这里可以根据世界信息调整地图参数
}

void CarlaMapAdapter::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received occupancy grid");
    
    // 转换占用栅格为 GridMap 格式
    current_grid_map_ = convertToGridMap(*msg);
    map_initialized_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Map converted and cached");
}

auto_msgs::msg::GridMap CarlaMapAdapter::convertToGridMap(const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
    auto_msgs::msg::GridMap grid_map;
    
    // 设置头信息
    grid_map.header.stamp = this->get_clock()->now();
    grid_map.header.frame_id = map_frame_;
    
    // 设置地图信息
    grid_map.resolution = occupancy_grid.info.resolution;
    grid_map.width = occupancy_grid.info.width;
    grid_map.height = occupancy_grid.info.height;
    
    // 设置原点
    grid_map.origin = occupancy_grid.info.origin;
    
    // 转换数据
    grid_map.data.reserve(occupancy_grid.data.size());
    for (const auto& cell : occupancy_grid.data) {
        int8_t value;
        if (cell == -1) {
            // 未知区域
            value = -1;
        } else if (cell >= 65) {
            // 障碍物 (标准占用栅格中 > 65 表示占用)
            value = 100;
        } else {
            // 自由空间
            value = 0;
        }
        grid_map.data.push_back(value);
    }
    
    return grid_map;
}

void CarlaMapAdapter::publishMapInfo() {
    if (!map_initialized_) {
        RCLCPP_DEBUG(this->get_logger(), "Map not initialized yet");
        return;
    }
    
    // 发布地图
    grid_map_pub_->publish(current_grid_map_);
    RCLCPP_DEBUG(this->get_logger(), "Published grid map");
}

} // namespace carla_integration

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<carla_integration::CarlaMapAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

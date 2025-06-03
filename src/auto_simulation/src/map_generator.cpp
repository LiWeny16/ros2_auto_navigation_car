#include "auto_simulation/map_generator.hpp"
#include <cmath>

namespace auto_simulation {

MapGenerator::MapGenerator(rclcpp::Node* node) 
    : node_(node), map_sequence_(0) {
    // 创建发布者
    map_pub_ = node_->create_publisher<auto_msgs::msg::GridMap>("grid_map", 10);
    map_json_pub_ = json_utils::create_json_publisher(node_, "grid_map", 10);
    
    RCLCPP_INFO(node_->get_logger(), "地图生成器已初始化");
}

auto_msgs::msg::GridMap MapGenerator::generateAndPublishMap() {
    // 创建地图
    auto_msgs::msg::GridMap map = createMap();
    
    // 增加地图序列号
    map_sequence_++;
    
    // 使用序列号作为随机种子，确保地图有变化但可重现
    std::mt19937 gen(map_sequence_ * 12345 + 67890);
    
    // 添加障碍物
    addRandomObstacles(map, gen);
    
    // 添加线性障碍物
    addLinearObstacles(map, gen);
    
    // 发布地图
    map_pub_->publish(map);
    
    // 同时发布JSON格式的地图
    json_utils::publish_as_json(map_json_pub_, map, "grid_map");
    
    RCLCPP_INFO(node_->get_logger(), "已发布地图 (序列: %d, 同时以JSON格式发布)", map_sequence_);
    
    // 保存当前地图
    current_map_ = map;
    
    return map;
}

const auto_msgs::msg::GridMap& MapGenerator::getCurrentMap() const {
    return current_map_;
}

int MapGenerator::getMapSequence() const {
    return map_sequence_;
}

auto_msgs::msg::GridMap MapGenerator::createMap() {
    auto_msgs::msg::GridMap map;
    map.header.stamp = node_->now();
    map.header.frame_id = "map";
    
    // 设置地图尺寸和分辨率
    map.width = 100;
    map.height = 100;
    map.resolution = 0.5;  // 每个网格0.5米
    
    // 设置地图原点
    map.origin.position.x = -25.0;
    map.origin.position.y = -25.0;
    map.origin.position.z = 0.0;
    map.origin.orientation.w = 1.0;
    
    // 初始化地图数据（所有单元格默认为0，表示可通行）
    map.data.resize(map.width * map.height, 0);
    
    return map;
}

void MapGenerator::addRandomObstacles(auto_msgs::msg::GridMap& map, std::mt19937& gen) {
    std::uniform_int_distribution<> dis_x(10, map.width - 10);
    std::uniform_int_distribution<> dis_y(10, map.height - 10);
    std::uniform_int_distribution<> dis_size(3, 8); // 减小障碍物尺寸
    
    // 添加随机障碍物（数量也会变化）
    int num_obstacles = 2 + (map_sequence_ % 3); // 2-4个障碍物（减少数量）
    for (int i = 0; i < num_obstacles; ++i) {
        int center_x = dis_x(gen);
        int center_y = dis_y(gen);
        int size = dis_size(gen);
        
        for (unsigned int dx = 0; dx <= static_cast<unsigned int>(size); ++dx) {
            for (unsigned int dy = 0; dy <= static_cast<unsigned int>(size); ++dy) {
                int x = center_x + static_cast<int>(dx) - size/2;
                int y = center_y + static_cast<int>(dy) - size/2;
                if (dx*dx + dy*dy <= static_cast<unsigned int>(size*size)/4) {  // 圆形障碍物
                    if (x >= 0 && static_cast<unsigned int>(x) < map.width && y >= 0 && static_cast<unsigned int>(y) < map.height) {
                        map.data[y * map.width + x] = 100;  // 100表示障碍物
                    }
                }
            }
        }
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "已添加 %d 个随机障碍物", num_obstacles);
}

void MapGenerator::addLinearObstacles(auto_msgs::msg::GridMap& map, std::mt19937& gen) {
    std::uniform_int_distribution<> dis_x(10, map.width - 10);
    std::uniform_int_distribution<> dis_y(10, map.height - 10);
    
    // 添加线性障碍物（墙壁）
    int num_walls = 1 + (map_sequence_ % 2); // 1-2个墙壁（减少数量）
    for (int i = 0; i < num_walls; ++i) {
        int start_x = dis_x(gen);
        int start_y = dis_y(gen);
        int end_x = dis_x(gen);
        int end_y = dis_y(gen);
        
        // 绘制线段
        int dx = std::abs(end_x - start_x);
        int dy = std::abs(end_y - start_y);
        int sx = (start_x < end_x) ? 1 : -1;
        int sy = (start_y < end_y) ? 1 : -1;
        int err = dx - dy;
        
        unsigned int x = static_cast<unsigned int>(start_x);
        unsigned int y = static_cast<unsigned int>(start_y);
        while (true) {
            if (x < map.width && y < map.height) {
                map.data[y * map.width + x] = 100;
            }
            
            if (x == static_cast<unsigned int>(end_x) && y == static_cast<unsigned int>(end_y)) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "已添加 %d 条墙壁障碍物", num_walls);
}

} // namespace auto_simulation

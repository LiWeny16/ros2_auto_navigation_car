#include "auto_perception/object_detector.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace auto_perception {

ObjectDetector::ObjectDetector() : next_object_id_(1) {}

std::vector<DetectedObject> ObjectDetector::detectObjects(const auto_msgs::msg::GridMap& map) {
    // 获取连接的组件（潜在的障碍物）
    auto components = findConnectedComponents(map);
    
    std::vector<DetectedObject> objects;
    objects.reserve(components.size());
    
    // 将每个组件转换为检测到的对象
    for (const auto& component : components) {
        if (component.size() > 3) {  // 过滤掉太小的组件（可能是噪声）
            auto object = componentToObject(component, map, next_object_id_++);
            objects.push_back(object);
        }
    }
    
    return objects;
}

std::vector<std::vector<std::pair<int, int>>> ObjectDetector::findConnectedComponents(
    const auto_msgs::msg::GridMap& map) {
    
    const int width = map.width;
    const int height = map.height;
    std::vector<bool> visited(width * height, false);
    std::vector<std::vector<std::pair<int, int>>> components;
    
    // 四个方向的偏移（上、右、下、左）
    const std::vector<std::pair<int, int>> directions = {
        {0, -1}, {1, 0}, {0, 1}, {-1, 0}
    };
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            
            // 如果单元格被占用且尚未访问
            if (map.data[index] == 100 && !visited[index]) {
                std::vector<std::pair<int, int>> component;
                std::queue<std::pair<int, int>> queue;
                
                // 标记为已访问
                visited[index] = true;
                queue.push({x, y});
                component.push_back({x, y});
                
                while (!queue.empty()) {
                    auto [cur_x, cur_y] = queue.front();
                    queue.pop();
                    
                    // 检查四个相邻的单元格
                    for (const auto& [dx, dy] : directions) {
                        int nx = cur_x + dx;
                        int ny = cur_y + dy;
                        int nindex = ny * width + nx;
                        
                        // 检查边界
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            // 如果相邻单元格被占用且尚未访问
                            if (map.data[nindex] == 100 && !visited[nindex]) {
                                visited[nindex] = true;
                                queue.push({nx, ny});
                                component.push_back({nx, ny});
                            }
                        }
                    }
                }
                
                components.push_back(component);
            }
        }
    }
    
    return components;
}

DetectedObject ObjectDetector::componentToObject(
    const std::vector<std::pair<int, int>>& component,
    const auto_msgs::msg::GridMap& map,
    int id) {
    
    // 计算组件的中心
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& [x, y] : component) {
        sum_x += x;
        sum_y += y;
    }
    double center_x = sum_x / component.size();
    double center_y = sum_y / component.size();
    
    // 计算组件的最小/最大边界
    int min_x = component[0].first, max_x = component[0].first;
    int min_y = component[0].second, max_y = component[0].second;
    
    for (const auto& [x, y] : component) {
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }
    
    // 计算宽度和长度（以网格单元为单位）
    double width_cells = max_x - min_x + 1;
    double length_cells = max_y - min_y + 1;
    
    // 转换为实际尺寸（米）
    double width = width_cells * map.resolution;
    double length = length_cells * map.resolution;
    
    // 转换为世界坐标
    geometry_msgs::msg::Pose pose;
    pose.position.x = map.origin.position.x + (center_x + 0.5) * map.resolution;
    pose.position.y = map.origin.position.y + (center_y + 0.5) * map.resolution;
    pose.position.z = 0.5;  // 假设高度为1米，中心在0.5米
    
    // 为对象分配一个随机方向（范围从0到2π）
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 2 * M_PI);
    double yaw = dis(gen);
    
    // 转换为四元数
    pose.orientation.w = cos(yaw / 2);
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(yaw / 2);
    
    // 创建并返回检测到的对象
    static const std::vector<std::string> classifications = {
        "vehicle", "pedestrian", "bicycle", "obstacle"
    };
    
    std::uniform_int_distribution<> class_dis(0, classifications.size() - 1);
    std::string classification = classifications[class_dis(gen)];
    
    std::uniform_real_distribution<> conf_dis(0.7, 1.0);
    double confidence = conf_dis(gen);
    
    return DetectedObject(id, pose, width, length, 1.0, classification, confidence);
}

visualization_msgs::msg::MarkerArray ObjectDetector::visualizeObjects(
    const std::vector<DetectedObject>& objects) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (const auto& obj : objects) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "detected_objects";
        marker.id = obj.id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位置和方向
        marker.pose = obj.pose;
        
        // 设置大小
        marker.scale.x = obj.length;
        marker.scale.y = obj.width;
        marker.scale.z = obj.height;
        
        // 根据对象类型设置颜色
        if (obj.classification == "vehicle") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (obj.classification == "pedestrian") {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else if (obj.classification == "bicycle") {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else {
            marker.color.r = 0.7;
            marker.color.g = 0.7;
            marker.color.b = 0.7;
        }
        marker.color.a = 0.8 * obj.confidence;  // 设置透明度
        
        // 设置生命周期
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        marker_array.markers.push_back(marker);
        
        // 添加文本标签
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = marker.header;
        text_marker.ns = "object_labels";
        text_marker.id = obj.id;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        text_marker.pose = obj.pose;
        text_marker.pose.position.z += obj.height / 2 + 0.5;  // 将文本置于对象上方
        
        text_marker.scale.z = 0.5;  // 文本高度
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        // 设置文本内容
        std::ostringstream ss;
        ss << obj.classification << " (" << std::fixed << std::setprecision(2) << obj.confidence << ")";
        text_marker.text = ss.str();
        
        text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        marker_array.markers.push_back(text_marker);
    }
    
    return marker_array;
}

} // namespace auto_perception

#ifndef OBJECT_DETECTOR_HPP_
#define OBJECT_DETECTOR_HPP_

#include <vector>
#include <memory>
#include "auto_msgs/msg/grid_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace auto_perception {

struct DetectedObject {
    int id;
    geometry_msgs::msg::Pose pose;
    double width;
    double length;
    double height;
    std::string classification;
    double confidence;
    
    DetectedObject(
        int id_,
        const geometry_msgs::msg::Pose& pose_,
        double width_ = 1.0,
        double length_ = 1.0,
        double height_ = 1.0,
        const std::string& classification_ = "unknown",
        double confidence_ = 1.0)
        : id(id_), pose(pose_), width(width_), length(length_), height(height_), 
          classification(classification_), confidence(confidence_) {}
};

class ObjectDetector {
public:
    ObjectDetector();
    virtual ~ObjectDetector() = default;

    // 从网格地图中检测障碍物
    std::vector<DetectedObject> detectObjects(const auto_msgs::msg::GridMap& map);
    
    // 可视化检测到的障碍物
    visualization_msgs::msg::MarkerArray visualizeObjects(const std::vector<DetectedObject>& objects);

private:
    // 查找连接的组件（障碍物聚类）
    std::vector<std::vector<std::pair<int, int>>> findConnectedComponents(
        const auto_msgs::msg::GridMap& map);
        
    // 获取障碍物的中心位置和大小
    DetectedObject componentToObject(
        const std::vector<std::pair<int, int>>& component,
        const auto_msgs::msg::GridMap& map,
        int id);

    int next_object_id_;
};

} // namespace auto_perception

#endif // OBJECT_DETECTOR_HPP_

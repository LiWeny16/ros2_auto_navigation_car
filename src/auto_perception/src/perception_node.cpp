#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "auto_perception/object_detector.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace auto_perception {

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
        // 创建对象检测器
        detector_ = std::make_unique<ObjectDetector>();
        
        // 创建订阅者
        map_sub_ = this->create_subscription<auto_msgs::msg::GridMap>(
            "grid_map", 10, std::bind(&PerceptionNode::mapCallback, this, _1));
        
        // 创建发布者
        objects_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "detected_objects_markers", 10);
        
        // 创建定时器用于处理感知任务
        perception_timer_ = this->create_wall_timer(
            100ms, std::bind(&PerceptionNode::perceptionUpdate, this));
        
        RCLCPP_INFO(this->get_logger(), "感知节点已启动");
    }

private:
    void mapCallback(const auto_msgs::msg::GridMap::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到地图，尺寸: %dx%d", msg->width, msg->height);
        current_map_ = *msg;
        have_map_ = true;
    }
    
    void perceptionUpdate() {
        if (!have_map_) {
            return;
        }
        
        // 从地图中检测对象
        auto detected_objects = detector_->detectObjects(current_map_);
        
        if (!detected_objects.empty()) {
            RCLCPP_INFO(this->get_logger(), "检测到 %zu 个对象", detected_objects.size());
            
            // 可视化检测到的对象
            auto markers = detector_->visualizeObjects(detected_objects);
            objects_vis_pub_->publish(markers);
            
            // 存储检测到的对象
            current_objects_ = detected_objects;
        }
    }
    
    std::unique_ptr<ObjectDetector> detector_;
    auto_msgs::msg::GridMap current_map_;
    std::vector<DetectedObject> current_objects_;
    bool have_map_ = false;
    
    rclcpp::Subscription<auto_msgs::msg::GridMap>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_vis_pub_;
    rclcpp::TimerBase::SharedPtr perception_timer_;
};

} // namespace auto_perception

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<auto_perception::PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

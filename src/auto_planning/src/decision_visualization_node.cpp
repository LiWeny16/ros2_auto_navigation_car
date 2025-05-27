#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace auto_planning {

class DecisionVisualizationNode : public rclcpp::Node {
public:
    DecisionVisualizationNode() : Node("decision_visualization_node") {
        // 创建订阅者
        path_sub_ = this->create_subscription<auto_msgs::msg::PlanningPath>(
            "planning_path", 10, std::bind(&DecisionVisualizationNode::pathCallback, this, _1));
        
        // 创建发布者
        decision_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "decision_markers", 10);
        
        // 创建定时器
        update_timer_ = this->create_wall_timer(
            500ms, std::bind(&DecisionVisualizationNode::updateVisualization, this));
        
        RCLCPP_INFO(this->get_logger(), "决策可视化节点已启动");
    }

private:
    void pathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到路径，点数: %zu", msg->points.size());
        current_path_ = *msg;
        have_path_ = true;
    }
    
    void updateVisualization() {
        if (!have_path_) {
            return;
        }
        
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 添加路径目标标记
        if (!current_path_.points.empty()) {
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = "map";
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "decision";
            goal_marker.id = 0;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 使用路径的最后一个点作为目标
            goal_marker.pose = current_path_.points.back().pose;
            
            // 设置大小
            goal_marker.scale.x = 1.5;
            goal_marker.scale.y = 1.5;
            goal_marker.scale.z = 1.5;
            
            // 设置颜色（绿色）
            goal_marker.color.r = 0.0;
            goal_marker.color.g = 1.0;
            goal_marker.color.b = 0.0;
            goal_marker.color.a = 0.8;
            
            marker_array.markers.push_back(goal_marker);
            
            // 添加目标文本标记
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = goal_marker.header;
            text_marker.ns = "decision_text";
            text_marker.id = 0;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose = goal_marker.pose;
            text_marker.pose.position.z += 2.0;
            
            text_marker.scale.z = 1.0;  // 文本高度
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            text_marker.text = "目标点";
            
            marker_array.markers.push_back(text_marker);
            
            // 添加路径安全走廊
            visualization_msgs::msg::Marker corridor_marker;
            corridor_marker.header.frame_id = "map";
            corridor_marker.header.stamp = this->now();
            corridor_marker.ns = "safety_corridor";
            corridor_marker.id = 0;
            corridor_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            corridor_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // 设置线条属性
            corridor_marker.scale.x = 0.2;  // 线宽
            corridor_marker.color.r = 0.0;
            corridor_marker.color.g = 0.8;
            corridor_marker.color.b = 0.8;
            corridor_marker.color.a = 0.5;
            
            // 在路径两侧创建安全走廊（简化版）
            const double corridor_width = 2.0;
            for (size_t i = 0; i < current_path_.points.size(); i += 5) {  // 每隔5个点采样一次
                if (i < current_path_.points.size()) {
                    // 获取路径点
                    const auto& point = current_path_.points[i];
                    tf2::Quaternion quat;
                    tf2::fromMsg(point.pose.orientation, quat);
                    double yaw = tf2::impl::getYaw(quat);
                    
                    // 计算垂直于路径方向的向量
                    double nx = -sin(yaw);
                    double ny = cos(yaw);
                    
                    // 左侧点
                    geometry_msgs::msg::Point left_point;
                    left_point.x = point.pose.position.x + corridor_width/2 * nx;
                    left_point.y = point.pose.position.y + corridor_width/2 * ny;
                    left_point.z = point.pose.position.z;
                    
                    corridor_marker.points.push_back(left_point);
                }
            }
            
            // 从终点返回
            for (int i = current_path_.points.size() - 1; i >= 0; i -= 5) {  // 每隔5个点采样一次
                if (i >= 0 && i < static_cast<int>(current_path_.points.size())) {
                    // 获取路径点
                    const auto& point = current_path_.points[i];
                    tf2::Quaternion quat;
                    tf2::fromMsg(point.pose.orientation, quat);
                    double yaw = tf2::impl::getYaw(quat);
                    
                    // 计算垂直于路径方向的向量
                    double nx = -sin(yaw);
                    double ny = cos(yaw);
                    
                    // 右侧点
                    geometry_msgs::msg::Point right_point;
                    right_point.x = point.pose.position.x - corridor_width/2 * nx;
                    right_point.y = point.pose.position.y - corridor_width/2 * ny;
                    right_point.z = point.pose.position.z;
                    
                    corridor_marker.points.push_back(right_point);
                }
            }
            
            // 闭合走廊
            if (!corridor_marker.points.empty()) {
                corridor_marker.points.push_back(corridor_marker.points.front());
            }
            
            marker_array.markers.push_back(corridor_marker);
        }
        
        // 发布标记
        decision_vis_pub_->publish(marker_array);
    }
    
    // 当前路径
    auto_msgs::msg::PlanningPath current_path_;
    bool have_path_ = false;
    
    // 订阅者
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_;
    
    // 发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr decision_vis_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr update_timer_;
};

} // namespace auto_planning

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<auto_planning::DecisionVisualizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

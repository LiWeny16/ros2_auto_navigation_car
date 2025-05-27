#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_control/pure_pursuit_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace auto_control {

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        // 创建控制器
        controller_ = std::make_unique<PurePursuitController>();
        controller_->setLookaheadDistance(3.0);
        controller_->setMaxVelocity(5.0);
        
        // 创建订阅者
        path_sub_ = this->create_subscription<auto_msgs::msg::PlanningPath>(
            "planning_path", 10, std::bind(&ControllerNode::pathCallback, this, _1));
        
        // 创建发布者
        cmd_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "control_markers", 10);
        
        // 初始化模拟的车辆状态
        vehicle_pose_.header.frame_id = "map";
        vehicle_pose_.pose.position.x = 0.0;
        vehicle_pose_.pose.position.y = 0.0;
        vehicle_pose_.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        vehicle_pose_.pose.orientation = tf2::toMsg(q);
        
        vehicle_velocity_.linear.x = 0.0;
        vehicle_velocity_.linear.y = 0.0;
        vehicle_velocity_.angular.z = 0.0;
        
        // 创建定时器用于控制循环和车辆状态更新
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&ControllerNode::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "控制节点已启动");
    }

private:
    void pathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到路径，点数: %zu", msg->points.size());
        controller_->setPath(*msg);
    }
    
    void controlLoop() {
        if (!controller_->hasPath()) {
            return;
        }
        
        // 计算控制命令
        auto cmd = controller_->calculateControl(vehicle_pose_.pose, vehicle_velocity_);
        
        // 更新车辆状态（简单运动学模型）
        updateVehicleState(cmd);
        
        // 可视化控制结果
        visualizeControl(cmd);
        
        // 检查是否到达目标
        if (controller_->reachedGoal()) {
            RCLCPP_INFO(this->get_logger(), "已到达目标位置！");
        }
    }
    
    void updateVehicleState(const ControlCommand& cmd) {
        // 获取当前状态
        double x = vehicle_pose_.pose.position.x;
        double y = vehicle_pose_.pose.position.y;
        tf2::Quaternion quat;
        tf2::fromMsg(vehicle_pose_.pose.orientation, quat);
        double yaw = tf2::impl::getYaw(quat);
        double v = vehicle_velocity_.linear.x;
        
        // 时间步长
        double dt = 0.05;  // 50ms
        
        // 计算加速度（简化）
        double acceleration = cmd.throttle * 2.0 - cmd.brake * 5.0;  // 简单的加速度模型
        
        // 更新速度（考虑最大/最小速度限制）
        v += acceleration * dt;
        v = std::max(0.0, std::min(v, 10.0));  // 速度范围：0-10 m/s
        
        // 更新位置
        double delta = cmd.steering_angle;  // 转向角
        double wheelbase = 2.7;  // 车辆轴距
        
        // 自行车模型
        double beta = atan(0.5 * tan(delta));  // 侧偏角
        double dx = v * cos(yaw + beta) * dt;
        double dy = v * sin(yaw + beta) * dt;
        double dyaw = v * tan(delta) * cos(beta) / wheelbase * dt;
        
        // 更新位置和方向
        x += dx;
        y += dy;
        yaw += dyaw;
        
        // 规范化角度到[-pi, pi]
        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw < -M_PI) yaw += 2.0 * M_PI;
        
        // 更新位置
        vehicle_pose_.header.stamp = this->now();
        vehicle_pose_.pose.position.x = x;
        vehicle_pose_.pose.position.y = y;
        
        // 更新方向
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        vehicle_pose_.pose.orientation = tf2::toMsg(q);
        
        // 更新速度
        vehicle_velocity_.linear.x = v * cos(yaw);
        vehicle_velocity_.linear.y = v * sin(yaw);
        vehicle_velocity_.angular.z = v * tan(delta) / wheelbase;
    }
    
    void visualizeControl(const ControlCommand& cmd) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 添加车辆主体标记（更大更明显）
        visualization_msgs::msg::Marker vehicle_marker;
        vehicle_marker.header.frame_id = "map";
        vehicle_marker.header.stamp = this->now();
        vehicle_marker.ns = "vehicle";
        vehicle_marker.id = 0;
        vehicle_marker.type = visualization_msgs::msg::Marker::CUBE;
        vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位置和方向
        vehicle_marker.pose = vehicle_pose_.pose;
        vehicle_marker.pose.position.z = 0.75;  // 抬高车辆，使其更明显
        
        // 设置车辆大小（更大）
        vehicle_marker.scale.x = 4.8;  // 长度
        vehicle_marker.scale.y = 2.2;  // 宽度
        vehicle_marker.scale.z = 1.8;  // 高度
        
        // 设置颜色（更亮的蓝色）
        vehicle_marker.color.r = 0.1;
        vehicle_marker.color.g = 0.3;
        vehicle_marker.color.b = 1.0;
        vehicle_marker.color.a = 0.9;
        
        marker_array.markers.push_back(vehicle_marker);
        
        // 添加车辆方向指示箭头
        visualization_msgs::msg::Marker direction_marker;
        direction_marker.header.frame_id = "map";
        direction_marker.header.stamp = this->now();
        direction_marker.ns = "vehicle_direction";
        direction_marker.id = 0;
        direction_marker.type = visualization_msgs::msg::Marker::ARROW;
        direction_marker.action = visualization_msgs::msg::Marker::ADD;
        
        direction_marker.pose = vehicle_pose_.pose;
        direction_marker.pose.position.z = 2.0;  // 在车辆上方
        
        // 箭头大小
        direction_marker.scale.x = 3.0;  // 长度
        direction_marker.scale.y = 0.5;  // 宽度
        direction_marker.scale.z = 0.5;  // 高度
        
        // 明亮的黄色箭头
        direction_marker.color.r = 1.0;
        direction_marker.color.g = 1.0;
        direction_marker.color.b = 0.0;
        direction_marker.color.a = 1.0;
        
        marker_array.markers.push_back(direction_marker);
        
        // 添加车辆轮廓标记
        visualization_msgs::msg::Marker outline_marker;
        outline_marker.header.frame_id = "map";
        outline_marker.header.stamp = this->now();
        outline_marker.ns = "vehicle_outline";
        outline_marker.id = 0;
        outline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        outline_marker.action = visualization_msgs::msg::Marker::ADD;
        
        outline_marker.pose = vehicle_pose_.pose;
        outline_marker.scale.x = 0.2;  // 线宽
        
        // 白色轮廓
        outline_marker.color.r = 1.0;
        outline_marker.color.g = 1.0;
        outline_marker.color.b = 1.0;
        outline_marker.color.a = 1.0;
        
        // 计算车辆四个角的位置
        tf2::Quaternion quat;
        tf2::fromMsg(vehicle_pose_.pose.orientation, quat);
        double yaw = tf2::impl::getYaw(quat);
        
        double half_length = 2.4;
        double half_width = 1.1;
        
        // 车辆四个角
        std::vector<std::pair<double, double>> corners = {
            {half_length, half_width},    // 右前
            {half_length, -half_width},   // 左前
            {-half_length, -half_width},  // 左后
            {-half_length, half_width},   // 右后
            {half_length, half_width}     // 回到起点闭合
        };
        
        for (const auto& [local_x, local_y] : corners) {
            geometry_msgs::msg::Point p;
            p.x = vehicle_pose_.pose.position.x + local_x * cos(yaw) - local_y * sin(yaw);
            p.y = vehicle_pose_.pose.position.y + local_x * sin(yaw) + local_y * cos(yaw);
            p.z = 0.1;
            outline_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(outline_marker);
        
        // 发布标记
        cmd_vis_pub_->publish(marker_array);
    }
    
    std::unique_ptr<PurePursuitController> controller_;
    geometry_msgs::msg::PoseStamped vehicle_pose_;
    geometry_msgs::msg::Twist vehicle_velocity_;
    
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cmd_vis_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

} // namespace auto_control

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<auto_control::ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

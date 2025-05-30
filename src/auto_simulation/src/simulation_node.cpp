#include <memory>
#include <string>
#include <vector>
#include <random>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace auto_simulation {

class SimulationNode : public rclcpp::Node {
public:
    SimulationNode() : Node("simulation_node") {
        // 创建发布者
        map_pub_ = this->create_publisher<auto_msgs::msg::GridMap>("grid_map", 10);
        vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        planning_request_pub_ = this->create_publisher<auto_msgs::msg::PlanningRequest>("planning_request", 10);
        
        // 创建订阅者
        path_sub_ = this->create_subscription<auto_msgs::msg::PlanningPath>(
            "planning_path", 10, std::bind(&SimulationNode::pathCallback, this, std::placeholders::_1));
        
        // 创建定时器
        map_timer_ = this->create_wall_timer(
            10s, std::bind(&SimulationNode::publishMap, this));
        
        request_timer_ = this->create_wall_timer(
            10s, std::bind(&SimulationNode::sendPlanningRequest, this));
        
        RCLCPP_INFO(this->get_logger(), "模拟节点已启动");
    }

private:
void publishMap() {
    auto_msgs::msg::GridMap map;
    map.header.stamp = this->now();
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
    
    // 添加一些随机障碍物
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(10, map.width - 10);
    std::uniform_int_distribution<> dis_y(10, map.height - 10);
    std::uniform_int_distribution<> dis_size(5, 15);
    
    // 添加几个随机形状的障碍物
    for (int i = 0; i < 5; ++i) {
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
    
    // 添加一些线性障碍物（墙壁）
    for (int i = 0; i < 3; ++i) {
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
    
    // 发布地图
    map_pub_->publish(map);
    RCLCPP_INFO(this->get_logger(), "已发布地图");
    
    // 保存当前地图用于路径规划
    current_map_ = map;
    
    // 可视化地图
    publishMapVisualization(map);
}
    void publishMapVisualization(const auto_msgs::msg::GridMap& map) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 创建障碍物可视化标记
        visualization_msgs::msg::Marker obstacle_marker;
        obstacle_marker.header = map.header;
        obstacle_marker.ns = "obstacles";
        obstacle_marker.id = 0;
        obstacle_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
        obstacle_marker.scale.x = map.resolution;
        obstacle_marker.scale.y = map.resolution;
        obstacle_marker.scale.z = 0.1;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 0.0;
        obstacle_marker.color.b = 0.0;
        obstacle_marker.color.a = 1.0;
        obstacle_marker.pose.orientation.w = 1.0;
        
        // 遍历地图添加障碍物
        for (size_t y = 0; y < map.height; ++y) {
            for (size_t x = 0; x < map.width; ++x) {
                if (map.data[y * map.width + x] == 100) {  // 障碍物
                    geometry_msgs::msg::Point p;
                    p.x = map.origin.position.x + (x + 0.5) * map.resolution;
                    p.y = map.origin.position.y + (y + 0.5) * map.resolution;
                    p.z = 0.05;
                    obstacle_marker.points.push_back(p);
                }
            }
        }
        
        marker_array.markers.push_back(obstacle_marker);
        vis_pub_->publish(marker_array);
    }
    
    void sendPlanningRequest() {
        if (current_map_.width == 0 || current_map_.height == 0) {
            RCLCPP_WARN(this->get_logger(), "尚未生成地图，无法发送规划请求");
            return;
        }
        
        auto_msgs::msg::PlanningRequest request;
        request.header.stamp = this->now();
        request.header.frame_id = "map";
        
        // 设置起点和终点
        std::random_device rd;
        std::mt19937 gen(rd());
        
        // 在地图有效区域内随机选择起点和终点
        bool valid_request = false;
        int max_attempts = 100;
        
        while (!valid_request && max_attempts > 0) {
            // 随机选择起点和终点在地图内的位置
            std::uniform_int_distribution<> dis_x(5, current_map_.width - 5);
            std::uniform_int_distribution<> dis_y(5, current_map_.height - 5);
            
            int start_x = dis_x(gen);
            int start_y = dis_y(gen);
            int goal_x = dis_x(gen);
            int goal_y = dis_y(gen);
            
            // 确保起点和终点是空闲的
            if (current_map_.data[start_y * current_map_.width + start_x] == 0 && 
                current_map_.data[goal_y * current_map_.width + goal_x] == 0) {
                
                // 确保起点和终点有一定距离
                double dx = start_x - goal_x;
                double dy = start_y - goal_y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance > 40.0) {  // 至少40个网格单元
                    // 设置起点
                    request.start.header = request.header;
                    request.start.pose.position.x = current_map_.origin.position.x + (start_x + 0.5) * current_map_.resolution;
                    request.start.pose.position.y = current_map_.origin.position.y + (start_y + 0.5) * current_map_.resolution;
                    request.start.pose.position.z = 0.0;
                    
                    // 随机朝向
                    std::uniform_real_distribution<> dis_angle(0, 2 * M_PI);
                    double start_angle = dis_angle(gen);
                    request.start.pose.orientation.w = std::cos(start_angle / 2.0);
                    request.start.pose.orientation.z = std::sin(start_angle / 2.0);
                    
                    // 设置终点
                    request.goal.header = request.header;
                    request.goal.pose.position.x = current_map_.origin.position.x + (goal_x + 0.5) * current_map_.resolution;
                    request.goal.pose.position.y = current_map_.origin.position.y + (goal_y + 0.5) * current_map_.resolution;
                    request.goal.pose.position.z = 0.0;
                    
                    double goal_angle = dis_angle(gen);
                    request.goal.pose.orientation.w = std::cos(goal_angle / 2.0);
                    request.goal.pose.orientation.z = std::sin(goal_angle / 2.0);
                    
                    valid_request = true;
                }
            }
            
            max_attempts--;
        }
        
        if (!valid_request) {
            RCLCPP_ERROR(this->get_logger(), "无法生成有效的规划请求，请重试");
            return;
        }
        
        // 随机选择规划器类型
        std::uniform_int_distribution<> dis_planner(0, 1);
        request.planner_type = (dis_planner(gen) == 0) ? "astar" : "hybrid_astar";
        
        // 是否考虑运动学约束（对Hybrid A*有效）
        request.consider_kinematic = (request.planner_type == "hybrid_astar");
        
        // 发布规划请求
        planning_request_pub_->publish(request);
        RCLCPP_INFO(this->get_logger(), "已发送规划请求，规划器类型: %s", request.planner_type.c_str());
        
        // 可视化起点和终点
        publishStartGoalVisualization(request.start, request.goal);
    }
    
    void publishStartGoalVisualization(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) {
        
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 创建起点标记
        visualization_msgs::msg::Marker start_marker;
        start_marker.header = start.header;
        start_marker.ns = "planning_points";
        start_marker.id = 1;
        start_marker.type = visualization_msgs::msg::Marker::ARROW;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.pose = start.pose;
        start_marker.scale.x = 2.0;  // 箭头长度
        start_marker.scale.y = 0.5;  // 箭头宽度
        start_marker.scale.z = 0.5;  // 箭头高度
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        
        // 创建终点标记
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header = goal.header;
        goal_marker.ns = "planning_points";
        goal_marker.id = 2;
        goal_marker.type = visualization_msgs::msg::Marker::ARROW;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose = goal.pose;
        goal_marker.scale.x = 2.0;
        goal_marker.scale.y = 0.5;
        goal_marker.scale.z = 0.5;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 1.0;
        goal_marker.color.a = 1.0;
        
        marker_array.markers.push_back(start_marker);
        marker_array.markers.push_back(goal_marker);
        vis_pub_->publish(marker_array);
    }
    
    void pathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到规划路径，长度: %.2f米，规划时间: %.2f秒",
                  msg->total_distance, msg->planning_time);
        
        // 可视化路径
        publishPathVisualization(*msg);
    }
    
    void publishPathVisualization(const auto_msgs::msg::PlanningPath& path) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 创建路径标记（更粗的线条）
        visualization_msgs::msg::Marker path_marker;
        path_marker.header = path.header;
        path_marker.ns = "planning_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.3;  // 增加线宽，使路径更明显
        
        // 根据规划器类型设置不同颜色
        if (path.planner_type == "astar") {
            path_marker.color.r = 0.0;
            path_marker.color.g = 0.4;
            path_marker.color.b = 1.0;
        } else {  // hybrid_astar
            path_marker.color.r = 0.0;
            path_marker.color.g = 1.0;
            path_marker.color.b = 1.0;
        }
        path_marker.color.a = 1.0;
        
        // 添加路径点
        for (const auto& point : path.points) {
            geometry_msgs::msg::Point p;
            p.x = point.pose.position.x;
            p.y = point.pose.position.y;
            p.z = 0.15;  // 更高一些，避免与地图重叠
            path_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(path_marker);
        
        // 为所有路径类型添加方向指示
        if (!path.points.empty()) {
            // 每隔几个点放一个箭头表示方向
            int arrow_interval = std::max(1, static_cast<int>(path.points.size() / 10));  // 大约10个箭头
            for (size_t i = 0; i < path.points.size(); i += arrow_interval) {
                visualization_msgs::msg::Marker arrow;
                arrow.header = path.header;
                arrow.ns = "path_direction";
                arrow.id = i;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose = path.points[i].pose;
                arrow.pose.position.z = 0.2;  // 稍微高于路径线
                
                arrow.scale.x = 1.2;  // 箭头长度
                arrow.scale.y = 0.3;  // 箭头宽度
                arrow.scale.z = 0.3;  // 箭头高度
                
                // 设置箭头颜色
                if (path.planner_type == "astar") {
                    arrow.color.r = 1.0;
                    arrow.color.g = 1.0;
                    arrow.color.b = 0.0;  // 黄色箭头
                } else {
                    arrow.color.r = 1.0;
                    arrow.color.g = 0.5;
                    arrow.color.b = 0.0;  // 橙色箭头
                }
                arrow.color.a = 0.8;
                
                marker_array.markers.push_back(arrow);
            }
            
            // 添加车辆形状可视化（沿路径显示车辆轮廓）
            int vehicle_interval = std::max(3, static_cast<int>(path.points.size() / 8));  // 大约8个车辆
            for (size_t i = 0; i < path.points.size(); i += vehicle_interval) {
                // 创建车辆轮廓标记
                visualization_msgs::msg::Marker vehicle_marker;
                vehicle_marker.header = path.header;
                vehicle_marker.ns = "vehicle_footprint";
                vehicle_marker.id = i;
                vehicle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
                vehicle_marker.scale.x = 0.15;  // 线宽
                
                // 设置车辆轮廓颜色
                vehicle_marker.color.r = 0.8;
                vehicle_marker.color.g = 0.2;
                vehicle_marker.color.b = 0.8;  // 紫色
                vehicle_marker.color.a = 0.7;
                
                // 车辆尺寸参数
                double vehicle_length = 4.5;
                double vehicle_width = 2.0;
                double half_length = vehicle_length / 2.0;
                double half_width = vehicle_width / 2.0;
                
                // 获取车辆姿态
                double x = path.points[i].pose.position.x;
                double y = path.points[i].pose.position.y;
                double theta = 2.0 * std::atan2(path.points[i].pose.orientation.z, 
                                               path.points[i].pose.orientation.w);
                
                // 计算车辆四个角点的世界坐标
                std::vector<std::pair<double, double>> corners = {
                    {half_length, half_width},    // 右前角
                    {half_length, -half_width},   // 左前角
                    {-half_length, -half_width},  // 左后角
                    {-half_length, half_width},   // 右后角
                    {half_length, half_width}     // 回到起点形成闭合
                };
                
                double cos_theta = std::cos(theta);
                double sin_theta = std::sin(theta);
                
                for (const auto& corner : corners) {
                    double local_x = corner.first;
                    double local_y = corner.second;
                    
                    // 旋转变换到世界坐标
                    double world_x = x + local_x * cos_theta - local_y * sin_theta;
                    double world_y = y + local_x * sin_theta + local_y * cos_theta;
                    
                    geometry_msgs::msg::Point p;
                    p.x = world_x;
                    p.y = world_y;
                    p.z = 0.25;  // 高于路径
                    vehicle_marker.points.push_back(p);
                }
                
                marker_array.markers.push_back(vehicle_marker);
            }
            
            // 添加起点和终点特殊标记
            visualization_msgs::msg::Marker start_marker;
            start_marker.header = path.header;
            start_marker.ns = "path_endpoints";
            start_marker.id = 0;
            start_marker.type = visualization_msgs::msg::Marker::SPHERE;
            start_marker.action = visualization_msgs::msg::Marker::ADD;
            start_marker.pose = path.points.front().pose;
            start_marker.pose.position.z = 0.3;
            
            start_marker.scale.x = 0.8;
            start_marker.scale.y = 0.8;
            start_marker.scale.z = 0.8;
            
            start_marker.color.r = 0.0;
            start_marker.color.g = 1.0;
            start_marker.color.b = 0.0;  // 绿色起点
            start_marker.color.a = 1.0;
            
            marker_array.markers.push_back(start_marker);
            
            // 终点标记
            visualization_msgs::msg::Marker goal_marker = start_marker;
            goal_marker.id = 1;
            goal_marker.pose = path.points.back().pose;
            goal_marker.pose.position.z = 0.3;
            
            goal_marker.color.r = 1.0;
            goal_marker.color.g = 0.0;
            goal_marker.color.b = 0.0;  // 红色终点
            
            marker_array.markers.push_back(goal_marker);
        }
        
        vis_pub_->publish(marker_array);
    }
    
    // 发布者
    rclcpp::Publisher<auto_msgs::msg::GridMap>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
    rclcpp::Publisher<auto_msgs::msg::PlanningRequest>::SharedPtr planning_request_pub_;
    
    // 订阅者
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr request_timer_;
    
    // 当前地图
    auto_msgs::msg::GridMap current_map_;
};

} // namespace auto_simulation

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<auto_simulation::SimulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

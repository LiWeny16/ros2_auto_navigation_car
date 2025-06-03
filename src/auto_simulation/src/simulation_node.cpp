#include <memory>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/empty.hpp"
#include "auto_simulation/json_utils.hpp"

using namespace std::chrono_literals;

namespace auto_simulation {

class SimulationNode : public rclcpp::Node {
public:
    SimulationNode() : Node("simulation_node") {
        // 获取参数
        this->declare_parameter("map_update_interval", 5.0);
        this->declare_parameter("planning_interval", 10.0);  // 保留但不使用
        
        map_update_interval_ = this->get_parameter("map_update_interval").as_double();
        
        // 创建发布者
        map_pub_ = this->create_publisher<auto_msgs::msg::GridMap>("grid_map", 10);
        vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        planning_request_pub_ = this->create_publisher<auto_msgs::msg::PlanningRequest>("planning_request", 10);
        path_clear_pub_ = this->create_publisher<std_msgs::msg::Empty>("clear_path", 10);
        
        // 创建JSON格式的发布者
        map_json_pub_ = json_utils::create_json_publisher(this, "grid_map", 10);
        planning_request_json_pub_ = json_utils::create_json_publisher(this, "planning_request", 10);
        
        // 创建订阅者
        path_sub_ = this->create_subscription<auto_msgs::msg::PlanningPath>(
            "planning_path", 10, std::bind(&SimulationNode::pathCallback, this, std::placeholders::_1));
        
        // 只创建地图发布定时器
        map_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(map_update_interval_), 
            std::bind(&SimulationNode::publishMapAndRequestPlanning, this));
        
        // 创建延迟规划请求定时器（单次触发）
        planning_delay_timer_ = this->create_wall_timer(
            100ms, std::bind(&SimulationNode::sendDelayedPlanningRequest, this));
        planning_delay_timer_->cancel(); // 初始时取消
        
        // 初始化状态
        map_sequence_ = 0;
        have_valid_start_goal_ = false;
        
        RCLCPP_INFO(this->get_logger(), "模拟节点已启动 - 地图更新间隔: %.1fs", map_update_interval_);
        RCLCPP_INFO(this->get_logger(), "地图发布后将立即触发路径规划请求");
    }

private:
    void publishMapAndRequestPlanning() {
        // 1. 先清理旧路径
        clearOldPath();
        
        // 2. 发布新地图
        publishMap();
        
        // 3. 延迟一小段时间后发送规划请求，确保地图已被处理
        scheduleDelayedPlanningRequest();
        
        RCLCPP_INFO(this->get_logger(), "地图发布 -> 清理旧路径 -> 准备发送规划请求");
    }
    
    void clearOldPath() {
        // 发布路径清理消息给规划节点
        std_msgs::msg::Empty clear_msg;
        path_clear_pub_->publish(clear_msg);
        
        // 只清理特定的路径可视化标记，不要删除所有
        visualization_msgs::msg::MarkerArray clear_markers;
        
        // 清理路径线条
        visualization_msgs::msg::Marker delete_path;
        delete_path.header.stamp = this->now();
        delete_path.header.frame_id = "map";
        delete_path.ns = "planning_path";
        delete_path.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(delete_path);
        
        // 清理路径方向箭头
        visualization_msgs::msg::Marker delete_arrows;
        delete_arrows.header.stamp = this->now();
        delete_arrows.header.frame_id = "map";
        delete_arrows.ns = "path_direction";
        delete_arrows.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(delete_arrows);
        
        // 清理车辆轮廓
        visualization_msgs::msg::Marker delete_vehicles;
        delete_vehicles.header.stamp = this->now();
        delete_vehicles.header.frame_id = "map";
        delete_vehicles.ns = "vehicle_footprint";
        delete_vehicles.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(delete_vehicles);
        
        // 清理路径端点
        visualization_msgs::msg::Marker delete_endpoints;
        delete_endpoints.header.stamp = this->now();
        delete_endpoints.header.frame_id = "map";
        delete_endpoints.ns = "path_endpoints";
        delete_endpoints.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(delete_endpoints);
        
        vis_pub_->publish(clear_markers);
        
        RCLCPP_DEBUG(this->get_logger(), "已清理旧路径可视化标记");
    }
    
    void scheduleDelayedPlanningRequest() {
        // 取消之前的定时器
        planning_delay_timer_->cancel();
        
        // 重新创建定时器，延迟发送规划请求
        planning_delay_timer_ = this->create_wall_timer(
            150ms, // 150ms延迟，确保地图已被规划节点处理
            std::bind(&SimulationNode::sendDelayedPlanningRequest, this));
    }
    
    void sendDelayedPlanningRequest() {
        planning_delay_timer_->cancel(); // 确保只执行一次
        
        RCLCPP_INFO(this->get_logger(), "发送延迟规划请求 (地图序列: %d)", map_sequence_);
        sendPlanningRequest();
    }

    void publishMap() {
        auto_msgs::msg::GridMap map;
        map.header.stamp = this->now();
        map.header.frame_id = "map";
        
        // 增加地图序列号
        map_sequence_++;
        
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
        
        // 使用序列号作为随机种子，确保地图有变化但可重现
        std::mt19937 gen(map_sequence_ * 12345 + 67890);
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
        
        // 发布地图
        map_pub_->publish(map);
        
        // 同时发布JSON格式的地图
        json_utils::publish_as_json(map_json_pub_, map, "grid_map");
        
        RCLCPP_INFO(this->get_logger(), "已发布地图 (序列: %d, 障碍物: %d, 墙壁: %d, 同时以JSON格式发布)", 
                   map_sequence_, num_obstacles, num_walls);
        
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
        
        // 车辆参数（与HybridAStarPlanner保持一致）
        double vehicle_length = 4.5;
        double vehicle_width = 2.0;
        double safety_margin = 0.2;
        
        // 计算车辆占用的网格数量（包括安全边距）
        double total_length = vehicle_length + 2 * safety_margin;
        double total_width = vehicle_width + 2 * safety_margin;
        int length_grids = static_cast<int>(std::ceil(total_length / current_map_.resolution));
        int width_grids = static_cast<int>(std::ceil(total_width / current_map_.resolution));
        
        // 确保有足够的边距避免边界问题
        int margin = std::max(length_grids, width_grids) / 2 + 2;
        
        // 设置起点和终点
        std::mt19937 gen(map_sequence_ * 54321 + 98765); // 使用不同的种子确保起点终点变化
        
        // 在地图有效区域内随机选择起点和终点
        bool valid_request = false;
        int max_attempts = 100;
        
        while (!valid_request && max_attempts > 0) {
            // 随机选择起点和终点在地图内的位置（考虑车辆尺寸）
            std::uniform_int_distribution<> dis_x(margin, current_map_.width - margin);
            std::uniform_int_distribution<> dis_y(margin, current_map_.height - margin);
            
            int start_x = dis_x(gen);
            int start_y = dis_y(gen);
            int goal_x = dis_x(gen);
            int goal_y = dis_y(gen);
            
            // 检查起点区域是否完全空闲
            bool start_valid = true;
            for (int dy = -width_grids/2; dy <= width_grids/2 && start_valid; ++dy) {
                for (int dx = -length_grids/2; dx <= length_grids/2 && start_valid; ++dx) {
                    int check_x = start_x + dx;
                    int check_y = start_y + dy;
                    if (check_x >= 0 && check_x < static_cast<int>(current_map_.width) &&
                        check_y >= 0 && check_y < static_cast<int>(current_map_.height)) {
                        if (current_map_.data[check_y * current_map_.width + check_x] != 0) {
                            start_valid = false;
                        }
                    } else {
                        start_valid = false;
                    }
                }
            }
            
            // 检查终点区域是否完全空闲
            bool goal_valid = true;
            for (int dy = -width_grids/2; dy <= width_grids/2 && goal_valid; ++dy) {
                for (int dx = -length_grids/2; dx <= length_grids/2 && goal_valid; ++dx) {
                    int check_x = goal_x + dx;
                    int check_y = goal_y + dy;
                    if (check_x >= 0 && check_x < static_cast<int>(current_map_.width) &&
                        check_y >= 0 && check_y < static_cast<int>(current_map_.height)) {
                        if (current_map_.data[check_y * current_map_.width + check_x] != 0) {
                            goal_valid = false;
                        }
                    } else {
                        goal_valid = false;
                    }
                }
            }
            
            if (start_valid && goal_valid) {
                // 确保起点和终点有足够距离
                double dx = start_x - goal_x;
                double dy = start_y - goal_y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance > 20.0) {  // 至少20个网格单元的距离
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
                    
                    // 保存当前起点终点
                    current_start_ = request.start;
                    current_goal_ = request.goal;
                    have_valid_start_goal_ = true;
                    
                    valid_request = true;
                    
                    RCLCPP_INFO(this->get_logger(), "选择起点: 网格(%d,%d) 世界(%.2f,%.2f)", 
                               start_x, start_y, request.start.pose.position.x, request.start.pose.position.y);
                    RCLCPP_INFO(this->get_logger(), "选择终点: 网格(%d,%d) 世界(%.2f,%.2f)", 
                               goal_x, goal_y, request.goal.pose.position.x, request.goal.pose.position.y);
                }
            }
            max_attempts--;
        }
        
        if (!valid_request) {
            RCLCPP_ERROR(this->get_logger(), "无法找到有效的起点和终点（考虑车辆尺寸后）");
            return;
        }
        
        // 设置规划器类型
        request.planner_type = "astar";
        
        // 发布规划请求
        planning_request_pub_->publish(request);
        
        // 同时发布JSON格式的规划请求
        json_utils::publish_as_json(planning_request_json_pub_, request, "planning_request");
        
        RCLCPP_INFO(this->get_logger(), "已发送规划请求 (地图序列: %d, 规划器: %s, 同时以JSON格式发布)", 
                   map_sequence_, request.planner_type.c_str());
        
        // 可视化起点和终点
        publishStartGoalVisualization(request.start, request.goal);
    }
    
    void publishStartGoalVisualization(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) {
        
        visualization_msgs::msg::MarkerArray marker_array;
        
        // 起点标记
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.stamp = this->now();
        start_marker.header.frame_id = "map";
        start_marker.ns = "start_goal";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.pose = start.pose;
        start_marker.pose.position.z = 0.5;
        
        start_marker.scale.x = 1.0;
        start_marker.scale.y = 1.0;
        start_marker.scale.z = 1.0;
        
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        
        marker_array.markers.push_back(start_marker);
        
        // 终点标记
        visualization_msgs::msg::Marker goal_marker = start_marker;
        goal_marker.id = 1;
        goal_marker.pose = goal.pose;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        
        marker_array.markers.push_back(goal_marker);
        
        // 起点箭头表示朝向
        visualization_msgs::msg::Marker start_arrow;
        start_arrow.header = start_marker.header;
        start_arrow.ns = "start_goal";
        start_arrow.id = 2;
        start_arrow.type = visualization_msgs::msg::Marker::ARROW;
        start_arrow.action = visualization_msgs::msg::Marker::ADD;
        start_arrow.pose = start.pose;
        start_arrow.pose.position.z = 0.3;
        
        start_arrow.scale.x = 2.0;
        start_arrow.scale.y = 0.5;
        start_arrow.scale.z = 0.5;
        
        start_arrow.color.r = 0.0;
        start_arrow.color.g = 0.8;
        start_arrow.color.b = 0.0;
        start_arrow.color.a = 0.8;
        
        marker_array.markers.push_back(start_arrow);
        
        // 终点箭头表示朝向
        visualization_msgs::msg::Marker goal_arrow = start_arrow;
        goal_arrow.id = 3;
        goal_arrow.pose = goal.pose;
        goal_arrow.color.r = 0.8;
        goal_arrow.color.g = 0.0;
        goal_arrow.color.b = 0.0;
        
        marker_array.markers.push_back(goal_arrow);
        
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
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr path_clear_pub_;
    
    // JSON格式的发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_json_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_request_json_pub_;
    
    // 订阅者
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr planning_delay_timer_;
    
    // 当前地图
    auto_msgs::msg::GridMap current_map_;
    
    // 状态变量
    int map_sequence_;
    bool have_valid_start_goal_;
    geometry_msgs::msg::PoseStamped current_start_;
    geometry_msgs::msg::PoseStamped current_goal_;
    double map_update_interval_;
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

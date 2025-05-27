#include "auto_planning/hybrid_a_star_planner.hpp"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace auto_planning {

HybridAStarPlanner::HybridAStarPlanner(double wheel_base, double vehicle_length, double vehicle_width)
    : wheel_base_(wheel_base), vehicle_length_(vehicle_length), vehicle_width_(vehicle_width) {}

auto_msgs::msg::PlanningPath HybridAStarPlanner::plan(
    const auto_msgs::msg::GridMap& map,
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
    
    auto start_time = std::chrono::steady_clock::now();
    
    // 获取起点和终点的位置和方向
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    // 从四元数提取yaw角度
    double start_theta = 2.0 * std::atan2(start.pose.orientation.z, start.pose.orientation.w);
    
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double goal_theta = 2.0 * std::atan2(goal.pose.orientation.z, goal.pose.orientation.w);
    
    // 检查起点和终点是否有效
    if (!isCollisionFree(map, start_x, start_y, start_theta) || !isCollisionFree(map, goal_x, goal_y, goal_theta)) {
        std::cerr << "起点或终点不可达。" << std::endl;
        return auto_msgs::msg::PlanningPath();
    }
    
    // 优先队列，用于A*算法的开放列表
    std::priority_queue<HybridNode, std::vector<HybridNode>, std::greater<HybridNode>> open_list;
    
    // 闭合列表，用于记录已经访问过的节点
    std::unordered_map<HybridNodeKey, bool, HybridNodeKeyHash> closed_list;
    
    // 创建起始节点
    double h = heuristic(start_x, start_y, goal_x, goal_y);
    auto start_node = std::make_shared<HybridNode>(start_x, start_y, start_theta, 0.0, h);
    
    // 将起始节点加入开放列表
    open_list.push(*start_node);
    
    std::shared_ptr<HybridNode> goal_node = nullptr;
    double goal_threshold = 0.5;  // 目标接近阈值（米）
    double angle_threshold = 0.2;  // 角度接近阈值（弧度）
    
    // Hybrid A*主循环
    while (!open_list.empty()) {
        // 取出f值最小的节点
        HybridNode current = open_list.top();
        open_list.pop();
        
        // 创建节点键，用于在闭合列表中查找
        auto current_grid = worldToGrid(map, current.x, current.y);
        HybridNodeKey key = {current_grid.first, current_grid.second, thetaToIndex(current.theta)};
        
        // 如果已经访问过该节点，跳过
        if (closed_list.find(key) != closed_list.end()) {
            continue;
        }
        
        // 将当前节点加入闭合列表
        closed_list[key] = true;
        
        // 检查是否到达目标附近
        double dx = current.x - goal_x;
        double dy = current.y - goal_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        double dtheta = std::abs(current.theta - goal_theta);
        while (dtheta > M_PI) dtheta = 2.0 * M_PI - dtheta;
        
        if (dist < goal_threshold && dtheta < angle_threshold) {
            goal_node = std::make_shared<HybridNode>(current);
            break;
        }
        
        // 生成下一步可能的状态
        auto next_states = getNextStates(current, map);
        
        // 对每个可能的下一状态进行处理
        for (const auto& next : next_states) {
            auto next_grid = worldToGrid(map, next.x, next.y);
            HybridNodeKey next_key = {next_grid.first, next_grid.second, thetaToIndex(next.theta)};
            
            // 如果该状态已经访问过，跳过
            if (closed_list.find(next_key) != closed_list.end()) {
                continue;
            }
            
            // 计算新节点的代价
            double new_g = current.g_cost + move_step_;
            double new_h = heuristic(next.x, next.y, goal_x, goal_y);
            
            // 创建新节点
            auto new_node = std::make_shared<HybridNode>(
                next.x, next.y, next.theta, new_g, new_h, next.steering, std::make_shared<HybridNode>(current));
            
            // 加入开放列表
            open_list.push(*new_node);
        }
    }
    
    // 如果没有找到路径
    if (!goal_node) {
        std::cerr << "无法找到路径。" << std::endl;
        return auto_msgs::msg::PlanningPath();
    }
    
    // 从目标节点回溯，构建路径
    std::vector<HybridNode> path_nodes;
    std::shared_ptr<HybridNode> current = goal_node;
    
    while (current) {
        path_nodes.push_back(*current);
        current = current->parent;
    }
    
    // 反转路径，使其从起点指向终点
    std::reverse(path_nodes.begin(), path_nodes.end());
    
    // 创建规划路径消息
    auto_msgs::msg::PlanningPath path;
    path.header = map.header;
    path.planner_type = "hybrid_astar";
    
    // 构建路径点
    double total_distance = 0.0;
    for (size_t i = 0; i < path_nodes.size(); ++i) {
        auto_msgs::msg::PathPoint point;
        
        // 设置位置
        point.pose.position.x = path_nodes[i].x;
        point.pose.position.y = path_nodes[i].y;
        point.pose.position.z = 0.0;
        
        // 设置方向（从theta转换为四元数）
        double theta = path_nodes[i].theta;
        point.pose.orientation.w = std::cos(theta / 2.0);
        point.pose.orientation.x = 0.0;
        point.pose.orientation.y = 0.0;
        point.pose.orientation.z = std::sin(theta / 2.0);
        
        // 设置转向角和曲率
        point.steering_angle = path_nodes[i].steering;
        if (std::abs(point.steering_angle) > 0.001) {
            point.curvature = std::tan(point.steering_angle) / wheel_base_;
        } else {
            point.curvature = 0.0;
        }
        
        // 计算速度（这里简单设置为常量）
        point.velocity = 5.0;  // 5 m/s
        
        // 计算行驶距离
        if (i > 0) {
            double dx = path_nodes[i].x - path_nodes[i-1].x;
            double dy = path_nodes[i].y - path_nodes[i-1].y;
            total_distance += std::sqrt(dx * dx + dy * dy);
        }
        
        path.points.push_back(point);
    }
    
    path.total_distance = total_distance;
    
    // 计算规划时间
    auto end_time = std::chrono::steady_clock::now();
    path.planning_time = std::chrono::duration<double>(end_time - start_time).count();
    
    return path;
}

bool HybridAStarPlanner::isValidNode(const auto_msgs::msg::GridMap& map, double x, double y) {
    // 转换为网格坐标
    auto grid = worldToGrid(map, x, y);
    int grid_x = grid.first;
    int grid_y = grid.second;
    
    // 检查是否在地图范围内
    if (grid_x < 0 || grid_x >= static_cast<int>(map.width) || 
        grid_y < 0 || grid_y >= static_cast<int>(map.height)) {
        return false;
    }
    
    // 检查是否是障碍物
    int index = grid_y * map.width + grid_x;
    if (index >= 0 && index < static_cast<int>(map.data.size())) {
        return map.data[index] != 100;
    }
    
    return false;
}

bool HybridAStarPlanner::isCollisionFree(const auto_msgs::msg::GridMap& map, double x, double y, double theta) {
    // 获取车辆四个角点的坐标
    auto corners = getVehicleCorners(x, y, theta);
    
    // 检查每个角点是否在有效区域内
    for (const auto& corner : corners) {
        if (!isValidNode(map, corner.first, corner.second)) {
            return false;
        }
    }
    
    // 检查车辆边缘的额外点以确保完整的碰撞检测
    double check_resolution = 0.2;  // 每0.2米检查一个点
    
    // 检查车辆长边（左侧和右侧）
    for (int i = 0; i < 2; ++i) {
        double start_x = corners[i].first;
        double start_y = corners[i].second;
        double end_x = corners[(i + 1) % 4].first;
        double end_y = corners[(i + 1) % 4].second;
        
        double dx = end_x - start_x;
        double dy = end_y - start_y;
        double length = std::sqrt(dx * dx + dy * dy);
        int num_checks = static_cast<int>(length / check_resolution) + 1;
        
        for (int j = 0; j <= num_checks; ++j) {
            double t = static_cast<double>(j) / static_cast<double>(num_checks);
            double check_x = start_x + t * dx;
            double check_y = start_y + t * dy;
            
            if (!isValidNode(map, check_x, check_y)) {
                return false;
            }
        }
    }
    
    // 检查车辆短边（前面和后面）
    for (int i = 1; i < 3; ++i) {
        double start_x = corners[i].first;
        double start_y = corners[i].second;
        double end_x = corners[(i + 1) % 4].first;
        double end_y = corners[(i + 1) % 4].second;
        
        double dx = end_x - start_x;
        double dy = end_y - start_y;
        double length = std::sqrt(dx * dx + dy * dy);
        int num_checks = static_cast<int>(length / check_resolution) + 1;
        
        for (int j = 0; j <= num_checks; ++j) {
            double t = static_cast<double>(j) / static_cast<double>(num_checks);
            double check_x = start_x + t * dx;
            double check_y = start_y + t * dy;
            
            if (!isValidNode(map, check_x, check_y)) {
                return false;
            }
        }
    }
    
    return true;
}

std::vector<std::pair<double, double>> HybridAStarPlanner::getVehicleCorners(double x, double y, double theta) {
    std::vector<std::pair<double, double>> corners;
    
    // 考虑安全边距的车辆尺寸
    double half_length = (vehicle_length_ + 2 * safety_margin_) / 2.0;
    double half_width = (vehicle_width_ + 2 * safety_margin_) / 2.0;
    
    // 车辆坐标系下的四个角点（车辆中心为原点）
    std::vector<std::pair<double, double>> local_corners = {
        {half_length, half_width},    // 右前角
        {half_length, -half_width},   // 左前角
        {-half_length, -half_width},  // 左后角
        {-half_length, half_width}    // 右后角
    };
    
    // 将车辆坐标系下的角点转换到世界坐标系
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    
    for (const auto& local_corner : local_corners) {
        double local_x = local_corner.first;
        double local_y = local_corner.second;
        
        // 旋转变换
        double world_x = x + local_x * cos_theta - local_y * sin_theta;
        double world_y = y + local_x * sin_theta + local_y * cos_theta;
        
        corners.emplace_back(world_x, world_y);
    }
    
    return corners;
}

double HybridAStarPlanner::heuristic(double x1, double y1, double x2, double y2) {
    // 使用欧几里得距离作为启发式
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

std::pair<int, int> HybridAStarPlanner::worldToGrid(
    const auto_msgs::msg::GridMap& map, double x, double y) {
    
    // 计算相对于地图原点的偏移
    double dx = x - map.origin.position.x;
    double dy = y - map.origin.position.y;
    
    // 转换为网格坐标
    int grid_x = static_cast<int>(std::round(dx / map.resolution));
    int grid_y = static_cast<int>(std::round(dy / map.resolution));
    
    return {grid_x, grid_y};
}

std::pair<double, double> HybridAStarPlanner::gridToWorld(
    const auto_msgs::msg::GridMap& map, int x, int y) {
    
    // 转换为世界坐标
    double world_x = map.origin.position.x + x * map.resolution;
    double world_y = map.origin.position.y + y * map.resolution;
    
    return {world_x, world_y};
}

int HybridAStarPlanner::thetaToIndex(double theta) {
    // 确保角度在[0, 2π)范围内
    while (theta < 0) theta += 2.0 * M_PI;
    while (theta >= 2.0 * M_PI) theta -= 2.0 * M_PI;
    
    // 将角度转换为索引（假设有angle_size_个离散角度）
    return static_cast<int>(std::round(theta / (2.0 * M_PI) * angle_size_)) % angle_size_;
}

double HybridAStarPlanner::indexToTheta(int index) {
    // 将索引转换回角度
    return static_cast<double>(index) / static_cast<double>(angle_size_) * 2.0 * M_PI;
}

std::vector<HybridNode> HybridAStarPlanner::getNextStates(
    const HybridNode& current, const auto_msgs::msg::GridMap& map) {
    
    std::vector<HybridNode> next_states;
    
    // 对每个可能的转向角度
    for (double steering : steering_angles_) {
        // 计算新的位置和方向
        double theta = current.theta;
        double x = current.x;
        double y = current.y;
        
        // 使用自行车模型进行运动学模拟
        if (std::abs(steering) < 0.001) {
            // 直线行驶
            x += move_step_ * std::cos(theta);
            y += move_step_ * std::sin(theta);
        } else {
            // 曲线行驶
            double turning_radius = wheel_base_ / std::tan(steering);
            double beta = move_step_ / turning_radius;
            x += turning_radius * (std::sin(theta + beta) - std::sin(theta));
            y += turning_radius * (std::cos(theta) - std::cos(theta + beta));
            theta += beta;
        }
        
        // 规范化角度到[0, 2π)
        while (theta < 0) theta += 2.0 * M_PI;
        while (theta >= 2.0 * M_PI) theta -= 2.0 * M_PI;
        
        // 检查新状态是否有效（使用车辆形状进行碰撞检测）
        if (isCollisionFree(map, x, y, theta)) {
            // 计算启发式代价（仅估计，实际代价在主循环中计算）
            double h = 0.0;
            next_states.emplace_back(x, y, theta, 0.0, h, steering);
        }
    }
    
    return next_states;
}

} // namespace auto_planning

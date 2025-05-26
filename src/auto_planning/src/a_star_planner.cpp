#include "auto_planning/a_star_planner.hpp"
#include <chrono>
#include <iostream>
#include <algorithm>

namespace auto_planning {

AStarPlanner::AStarPlanner() {}

auto_msgs::msg::PlanningPath AStarPlanner::plan(
    const auto_msgs::msg::GridMap& map,
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
    
    auto start_time = std::chrono::steady_clock::now();
    
    // 将起点和终点转换为网格坐标
    auto start_grid = worldToGrid(map, start.pose);
    auto goal_grid = worldToGrid(map, goal.pose);
    
    // 如果起点或终点不在地图内或是障碍物，返回空路径
    if (!isValidNode(map, start_grid.first, start_grid.second) || 
        !isValidNode(map, goal_grid.first, goal_grid.second)) {
        std::cerr << "起点或终点不可达。" << std::endl;
        return auto_msgs::msg::PlanningPath();
    }
    
    // 用于A*算法的优先队列
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    
    // 用于快速查找已访问节点的哈希表
    std::unordered_map<int, std::unordered_map<int, bool>> closed_list;
    
    // 初始化起点
    auto h_cost = heuristic(start_grid.first, start_grid.second, goal_grid.first, goal_grid.second);
    auto start_node = std::make_shared<Node>(start_grid.first, start_grid.second, 0.0, h_cost);
    
    // 将起点加入开放列表
    open_list.push(*start_node);
    
    std::shared_ptr<Node> goal_node = nullptr;
    
    // A*主循环
    while (!open_list.empty()) {
        // 取出f值最小的节点
        Node current = open_list.top();
        open_list.pop();
        
        // 如果已经处理过这个节点，跳过
        if (closed_list[current.x][current.y]) {
            continue;
        }
        
        // 将当前节点标记为已访问
        closed_list[current.x][current.y] = true;
        
        // 检查是否到达目标点附近
        if (std::abs(current.x - goal_grid.first) <= 1 && 
            std::abs(current.y - goal_grid.second) <= 1) {
            goal_node = std::make_shared<Node>(current);
            break;
        }
        
        // 检查8个方向的邻居节点
        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;
            
            // 检查邻居节点是否有效且未访问过
            if (isValidNode(map, nx, ny) && !closed_list[nx][ny]) {
                // 计算从起点到该邻居的代价
                double move_cost = (dir.first != 0 && dir.second != 0) ? 1.414 : 1.0; // 对角线移动代价为√2
                double new_g_cost = current.g_cost + move_cost;
                
                // 计算该邻居到目标的启发式代价
                double new_h_cost = heuristic(nx, ny, goal_grid.first, goal_grid.second);
                
                // 创建新节点
                auto new_node = std::make_shared<Node>(
                    nx, ny, new_g_cost, new_h_cost, std::make_shared<Node>(current));
                
                // 将新节点加入开放列表
                open_list.push(*new_node);
            }
        }
    }
    
    // 如果没有找到路径
    if (!goal_node) {
        std::cerr << "无法找到路径。" << std::endl;
        return auto_msgs::msg::PlanningPath();
    }
    
    // 重建路径（从目标回溯到起点）
    std::vector<std::pair<int, int>> grid_path;
    std::shared_ptr<Node> current = goal_node;
    
    while (current) {
        grid_path.emplace_back(current->x, current->y);
        current = current->parent;
    }
    
    // 反转路径，使其从起点指向终点
    std::reverse(grid_path.begin(), grid_path.end());
    
    // 创建规划路径消息
    auto_msgs::msg::PlanningPath path;
    path.header = map.header;
    path.planner_type = "astar";
    
    // 将网格路径转换为世界坐标路径
    double total_distance = 0.0;
    geometry_msgs::msg::Pose prev_pose;
    bool first = true;
    
    for (const auto& grid_point : grid_path) {
        auto_msgs::msg::PathPoint point;
        point.pose = gridToWorld(map, grid_point.first, grid_point.second);
        
        // 计算行驶距离
        if (!first) {
            double dx = point.pose.position.x - prev_pose.position.x;
            double dy = point.pose.position.y - prev_pose.position.y;
            total_distance += std::sqrt(dx * dx + dy * dy);
        } else {
            first = false;
        }
        
        prev_pose = point.pose;
        path.points.push_back(point);
    }
    
    path.total_distance = total_distance;
    
    // 计算规划时间
    auto end_time = std::chrono::steady_clock::now();
    path.planning_time = std::chrono::duration<double>(end_time - start_time).count();
    
    return path;
}

std::pair<int, int> AStarPlanner::worldToGrid(
    const auto_msgs::msg::GridMap& map, 
    const geometry_msgs::msg::Pose& pose) {
    
    // 计算相对于地图原点的偏移
    double dx = pose.position.x - map.origin.position.x;
    double dy = pose.position.y - map.origin.position.y;
    
    // 转换为网格坐标（四舍五入到最近的整数）
    int grid_x = static_cast<int>(std::round(dx / map.resolution));
    int grid_y = static_cast<int>(std::round(dy / map.resolution));
    
    return {grid_x, grid_y};
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(
    const auto_msgs::msg::GridMap& map, 
    int x, int y) {
    
    geometry_msgs::msg::Pose pose;
    
    // 转换为世界坐标
    pose.position.x = map.origin.position.x + x * map.resolution;
    pose.position.y = map.origin.position.y + y * map.resolution;
    pose.position.z = 0.0;
    
    // 假设无旋转
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    
    return pose;
}

bool AStarPlanner::isValidNode(const auto_msgs::msg::GridMap& map, int x, int y) {
    // 检查是否在地图范围内
    if (x < 0 || x >= static_cast<int>(map.width) || 
        y < 0 || y >= static_cast<int>(map.height)) {
        return false;
    }
    
    // 检查是否是障碍物（假设值为100表示障碍物）
    int index = y * map.width + x;
    if (index >= 0 && index < static_cast<int>(map.data.size())) {
        return map.data[index] != 100;
    }
    
    return false;
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    // 使用欧几里得距离作为启发式
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace auto_planning

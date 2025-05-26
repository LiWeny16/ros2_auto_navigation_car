#include "auto_planning/optimized_a_star_planner.hpp"
#include <chrono>
#include <iostream>
#include <algorithm>
#include <future>
#include <cmath>

namespace auto_planning {

OptimizedAStarPlanner::OptimizedAStarPlanner() {}

auto_msgs::msg::PlanningPath OptimizedAStarPlanner::plan(
    const auto_msgs::msg::GridMap& map,
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    const AStarConfig& config) {
    
    auto start_time = std::chrono::steady_clock::now();
    
    // 设置节点池回收选项
    node_pool_.setUseRecycling(config.use_node_recycling);
    
    // 将起点和终点转换为网格坐标
    auto start_grid = worldToGrid(map, start.pose, config.grid_resolution);
    auto goal_grid = worldToGrid(map, goal.pose, config.grid_resolution);
    
    // 如果起点或终点不在地图内或是障碍物，返回空路径
    if (!isValidNode(map, start_grid.first, start_grid.second) || 
        !isValidNode(map, goal_grid.first, goal_grid.second)) {
        std::cerr << "起点或终点不可达。" << std::endl;
        return auto_msgs::msg::PlanningPath();
    }
    
    // 根据配置选择搜索方法
    std::shared_ptr<OptimizedNode> goal_node = nullptr;
    
    if (config.enable_parallel_search) {
        goal_node = searchMultiThreaded(map, start_grid, goal_grid, config);
    } else {
        goal_node = searchSingleThreaded(map, start_grid, goal_grid, config);
    }
    
    // 检查是否找到路径或超时
    auto current_time = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(current_time - start_time).count();
    
    if (elapsed > config.planning_timeout) {
        std::cerr << "规划超时。" << std::endl;
        return auto_msgs::msg::PlanningPath();
    }
    
    // 如果没有找到路径
    if (!goal_node) {
        std::cerr << "无法找到路径。" << std::endl;
        return auto_msgs::msg::PlanningPath();
    }
    
    // 重建路径（从目标回溯到起点）
    std::vector<std::pair<int, int>> grid_path;
    std::shared_ptr<OptimizedNode> current = goal_node;
    
    while (current) {
        grid_path.emplace_back(current->x, current->y);
        current = current->parent;
    }
    
    // 反转路径，使其从起点指向终点
    std::reverse(grid_path.begin(), grid_path.end());
    
    // 创建规划路径消息
    auto_msgs::msg::PlanningPath path;
    path.header = map.header;
    path.planner_type = "optimized_astar";
    
    // 将网格路径转换为世界坐标路径
    double total_distance = 0.0;
    geometry_msgs::msg::Pose prev_pose;
    bool first = true;
    
    for (const auto& grid_point : grid_path) {
        auto_msgs::msg::PathPoint point;
        point.pose = gridToWorld(map, grid_point.first, grid_point.second, config.grid_resolution);
        
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

std::shared_ptr<OptimizedNode> OptimizedAStarPlanner::searchSingleThreaded(
    const auto_msgs::msg::GridMap& map,
    const std::pair<int, int>& start_grid,
    const std::pair<int, int>& goal_grid,
    const AStarConfig& config) {
    
    // 用于A*算法的优先队列
    std::priority_queue<OptimizedNode, std::vector<OptimizedNode>, std::greater<OptimizedNode>> open_list;
    
    // 用于快速查找已访问节点的哈希表
    std::unordered_map<NodeKey, bool, NodeKeyHash> closed_list;
    
    // 用于快速查找开放列表中的节点
    std::unordered_map<NodeKey, double, NodeKeyHash> open_map;
    
    // 初始化起点
    double h_cost;
    if (config.use_improved_heuristic) {
        h_cost = heuristicImproved(start_grid.first, start_grid.second, 
                                 goal_grid.first, goal_grid.second, map);
    } else {
        h_cost = heuristicEuclidean(start_grid.first, start_grid.second, 
                                  goal_grid.first, goal_grid.second);
    }
    
    auto start_node = node_pool_.createNode(start_grid.first, start_grid.second, 0.0, h_cost);
    
    // 将起点加入开放列表
    open_list.push(*start_node);
    open_map[{start_node->x, start_node->y}] = 0.0;
    
    std::shared_ptr<OptimizedNode> goal_node = nullptr;
    int iterations = 0;
    
    // A*主循环
    while (!open_list.empty() && iterations < config.max_iterations) {
        iterations++;
        
        // 取出f值最小的节点
        OptimizedNode current = open_list.top();
        open_list.pop();
        
        NodeKey current_key = {current.x, current.y};
        
        // 从open_map中移除
        open_map.erase(current_key);
        
        // 如果已经处理过这个节点，跳过
        if (closed_list[current_key]) {
            continue;
        }
        
        // 将当前节点标记为已访问
        closed_list[current_key] = true;
        
        // 检查是否到达目标点附近
        double dx = current.x - goal_grid.first;
        double dy = current.y - goal_grid.second;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist <= config.goal_tolerance) {
            goal_node = node_pool_.createNode(current.x, current.y, current.g_cost, 
                                           current.h_cost, current.parent);
            break;
        }
        
        // 检查8个方向的邻居节点
        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;
            NodeKey neighbor_key = {nx, ny};
            
            // 检查邻居节点是否有效且未访问过
            if (isValidNode(map, nx, ny) && !closed_list[neighbor_key]) {
                // 计算从起点到该邻居的代价
                double move_cost = (dir.first != 0 && dir.second != 0) ? 1.414 : 1.0; // 对角线移动代价为√2
                double new_g_cost = current.g_cost + move_cost;
                
                // 如果该节点已在开放列表中且新路径不更优，跳过
                auto it = open_map.find(neighbor_key);
                if (it != open_map.end() && new_g_cost >= it->second) {
                    continue;
                }
                
                // 计算该邻居到目标的启发式代价
                double new_h_cost;
                if (config.use_improved_heuristic) {
                    new_h_cost = heuristicImproved(nx, ny, goal_grid.first, goal_grid.second, map);
                } else {
                    new_h_cost = heuristicEuclidean(nx, ny, goal_grid.first, goal_grid.second);
                }
                
                // 创建新节点
                auto parent = node_pool_.createNode(current.x, current.y, current.g_cost, 
                                                 current.h_cost, current.parent);
                auto new_node = node_pool_.createNode(nx, ny, new_g_cost, new_h_cost, parent);
                
                // 更新开放列表
                open_list.push(*new_node);
                open_map[neighbor_key] = new_g_cost;
            }
        }
    }
    
    return goal_node;
}

std::shared_ptr<OptimizedNode> OptimizedAStarPlanner::searchMultiThreaded(
    const auto_msgs::msg::GridMap& map,
    const std::pair<int, int>& start_grid,
    const std::pair<int, int>& goal_grid,
    const AStarConfig& config) {
    
    // 确定线程数量（根据处理器核心数）
    unsigned int num_threads = std::thread::hardware_concurrency();
    num_threads = std::max(1u, std::min(num_threads, 8u));  // 限制在1-8线程之间
    
    // 启动多个方向的搜索
    std::vector<std::future<std::shared_ptr<OptimizedNode>>> futures;
    std::atomic<bool> found_flag(false);
    
    for (unsigned int i = 0; i < num_threads; ++i) {
        futures.push_back(std::async(std::launch::async, 
            &OptimizedAStarPlanner::searchDirectional, this, 
            std::ref(map), start_grid, goal_grid, config, 
            i, num_threads, std::ref(found_flag)));
    }
    
    // 等待任一线程找到路径或所有线程完成
    std::shared_ptr<OptimizedNode> result = nullptr;
    for (auto& future : futures) {
        auto node = future.get();
        if (node) {
            // 如果这个线程找到了路径
            if (!result || node->f_cost < result->f_cost) {
                result = node;
            }
        }
    }
    
    return result;
}

std::shared_ptr<OptimizedNode> OptimizedAStarPlanner::searchDirectional(
    const auto_msgs::msg::GridMap& map,
    const std::pair<int, int>& start_grid,
    const std::pair<int, int>& goal_grid,
    const AStarConfig& config,
    int direction_offset,
    int direction_count,
    std::atomic<bool>& found_flag) {
    
    // 用于A*算法的优先队列
    std::priority_queue<OptimizedNode, std::vector<OptimizedNode>, std::greater<OptimizedNode>> open_list;
    
    // 用于快速查找已访问节点的哈希表
    std::unordered_map<NodeKey, bool, NodeKeyHash> closed_list;
    
    // 用于快速查找开放列表中的节点
    std::unordered_map<NodeKey, double, NodeKeyHash> open_map;
    
    // 获取搜索方向的子集
    std::vector<std::pair<int, int>> thread_directions;
    for (size_t i = direction_offset; i < directions.size(); i += direction_count) {
        thread_directions.push_back(directions[i]);
    }
    
    // 初始化起点
    double h_cost;
    if (config.use_improved_heuristic) {
        h_cost = heuristicImproved(start_grid.first, start_grid.second, 
                                 goal_grid.first, goal_grid.second, map);
    } else {
        h_cost = heuristicEuclidean(start_grid.first, start_grid.second, 
                                  goal_grid.first, goal_grid.second);
    }
    
    auto start_node = node_pool_.createNode(start_grid.first, start_grid.second, 0.0, h_cost);
    
    // 将起点加入开放列表
    open_list.push(*start_node);
    open_map[{start_node->x, start_node->y}] = 0.0;
    
    std::shared_ptr<OptimizedNode> goal_node = nullptr;
    int iterations = 0;
    int max_iterations = config.max_iterations / direction_count;
    
    // A*主循环
    while (!open_list.empty() && iterations < max_iterations && !found_flag.load()) {
        iterations++;
        
        // 取出f值最小的节点
        OptimizedNode current = open_list.top();
        open_list.pop();
        
        NodeKey current_key = {current.x, current.y};
        
        // 从open_map中移除
        open_map.erase(current_key);
        
        // 如果已经处理过这个节点，跳过
        if (closed_list[current_key]) {
            continue;
        }
        
        // 将当前节点标记为已访问
        closed_list[current_key] = true;
        
        // 检查是否到达目标点附近
        double dx = current.x - goal_grid.first;
        double dy = current.y - goal_grid.second;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist <= config.goal_tolerance) {
            goal_node = node_pool_.createNode(current.x, current.y, current.g_cost, 
                                           current.h_cost, current.parent);
            found_flag.store(true);
            break;
        }
        
        // 检查所有可能的方向
        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;
            NodeKey neighbor_key = {nx, ny};
            
            // 检查邻居节点是否有效且未访问过
            if (isValidNode(map, nx, ny) && !closed_list[neighbor_key]) {
                // 计算从起点到该邻居的代价
                double move_cost = (dir.first != 0 && dir.second != 0) ? 1.414 : 1.0; // 对角线移动代价为√2
                double new_g_cost = current.g_cost + move_cost;
                
                // 如果该节点已在开放列表中且新路径不更优，跳过
                auto it = open_map.find(neighbor_key);
                if (it != open_map.end() && new_g_cost >= it->second) {
                    continue;
                }
                
                // 计算该邻居到目标的启发式代价
                double new_h_cost;
                if (config.use_improved_heuristic) {
                    new_h_cost = heuristicImproved(nx, ny, goal_grid.first, goal_grid.second, map);
                } else {
                    new_h_cost = heuristicEuclidean(nx, ny, goal_grid.first, goal_grid.second);
                }
                
                // 创建新节点
                auto parent = node_pool_.createNode(current.x, current.y, current.g_cost, 
                                                 current.h_cost, current.parent);
                auto new_node = node_pool_.createNode(nx, ny, new_g_cost, new_h_cost, parent);
                
                // 更新开放列表
                open_list.push(*new_node);
                open_map[neighbor_key] = new_g_cost;
            }
        }
    }
    
    return goal_node;
}

std::pair<int, int> OptimizedAStarPlanner::worldToGrid(
    const auto_msgs::msg::GridMap& map, 
    const geometry_msgs::msg::Pose& pose,
    int resolution_multiplier) {
    
    // 计算相对于地图原点的偏移
    double dx = pose.position.x - map.origin.position.x;
    double dy = pose.position.y - map.origin.position.y;
    
    // 转换为网格坐标，考虑分辨率倍数
    int grid_x = static_cast<int>(std::round(dx / (map.resolution * resolution_multiplier)));
    int grid_y = static_cast<int>(std::round(dy / (map.resolution * resolution_multiplier)));
    
    return {grid_x, grid_y};
}

geometry_msgs::msg::Pose OptimizedAStarPlanner::gridToWorld(
    const auto_msgs::msg::GridMap& map, 
    int x, int y,
    int resolution_multiplier) {
    
    geometry_msgs::msg::Pose pose;
    
    // 转换为世界坐标，考虑分辨率倍数
    pose.position.x = map.origin.position.x + x * map.resolution * resolution_multiplier;
    pose.position.y = map.origin.position.y + y * map.resolution * resolution_multiplier;
    pose.position.z = 0.0;
    
    // 假设无旋转
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    
    return pose;
}

bool OptimizedAStarPlanner::isValidNode(const auto_msgs::msg::GridMap& map, int x, int y) {
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

double OptimizedAStarPlanner::heuristicEuclidean(int x1, int y1, int x2, int y2) {
    // 使用欧几里得距离作为启发式
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double OptimizedAStarPlanner::heuristicImproved(
    int x1, int y1, int x2, int y2, const auto_msgs::msg::GridMap& map) {
    
    // 基础欧几里得距离
    double euclidean = heuristicEuclidean(x1, y1, x2, y2);
    
    // 曼哈顿距离
    double manhattan = std::abs(x2 - x1) + std::abs(y2 - y1);
    
    // 混合启发式函数，考虑障碍物密度
    // 可以根据实际情况调整权重
    double weight_euclidean = 1.0;
    double weight_manhattan = 0.5;
    
    // 基本混合
    double basic_h = weight_euclidean * euclidean + weight_manhattan * manhattan;
    
    // 根据起点和终点之间的障碍物密度调整启发式
    // 这是一个简化的实现，实际中可能需要更复杂的障碍物分析
    int obstacle_count = 0;
    int total_cells = 0;
    
    // 采样起点和终点之间的区域
    int min_x = std::min(x1, x2);
    int max_x = std::max(x1, x2);
    int min_y = std::min(y1, y2);
    int max_y = std::max(y1, y2);
    
    // 限制采样区域大小，避免过度计算
    int sample_limit = 100;
    if (max_x - min_x > sample_limit) {
        max_x = min_x + sample_limit;
    }
    if (max_y - min_y > sample_limit) {
        max_y = min_y + sample_limit;
    }
    
    // 计算障碍物密度
    for (int i = min_x; i <= max_x; ++i) {
        for (int j = min_y; j <= max_y; ++j) {
            if (i >= 0 && i < static_cast<int>(map.width) && 
                j >= 0 && j < static_cast<int>(map.height)) {
                
                total_cells++;
                
                int index = j * map.width + i;
                if (index >= 0 && index < static_cast<int>(map.data.size()) && 
                    map.data[index] == 100) {
                    obstacle_count++;
                }
            }
        }
    }
    
    // 障碍物密度因子
    double obstacle_factor = 1.0;
    if (total_cells > 0) {
        double density = static_cast<double>(obstacle_count) / total_cells;
        obstacle_factor = 1.0 + density * 2.0;  // 障碍物越多，启发式值越大
    }
    
    return basic_h * obstacle_factor;
}

} // namespace auto_planning

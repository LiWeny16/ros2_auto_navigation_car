#include "auto_integration_test/performance_optimizer.hpp"
#include <fstream>
#include <chrono>
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>
#include <sys/resource.h>

namespace auto_integration_test {

using namespace std::chrono_literals;

PerformanceOptimizer::PerformanceOptimizer() 
    : Node("performance_optimizer"), received_result_(false) {
    // 创建发布者
    map_pub_ = this->create_publisher<auto_msgs::msg::GridMap>("grid_map", 10);
    request_pub_ = this->create_publisher<auto_msgs::msg::PlanningRequest>("planning_request", 10);
    
    // 创建订阅者
    path_sub_ = this->create_subscription<auto_msgs::msg::PlanningPath>(
        "planning_path", 10, std::bind(&PerformanceOptimizer::planningPathCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "性能优化器已启动");
    
    // 等待一秒，确保ROS2节点连接
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void PerformanceOptimizer::optimizeAStar() {
    RCLCPP_INFO(this->get_logger(), "开始优化A*算法...");
    
    // 创建不同的配置进行测试
    std::vector<OptimizerConfig> configs;
    
    // 基准配置
    OptimizerConfig base_config;
    base_config.enable_parallel_search = false;
    base_config.use_improved_heuristic = false;
    base_config.max_iterations = 10000;
    base_config.goal_tolerance = 0.5;
    base_config.grid_resolution = 1;
    base_config.use_node_recycling = false;
    base_config.planning_timeout = 5.0;
    configs.push_back(base_config);
    
    // 启用并行搜索
    OptimizerConfig parallel_config = base_config;
    parallel_config.enable_parallel_search = true;
    configs.push_back(parallel_config);
    
    // 使用改进的启发式函数
    OptimizerConfig heuristic_config = base_config;
    heuristic_config.use_improved_heuristic = true;
    configs.push_back(heuristic_config);
    
    // 降低网格分辨率（更粗糙但更快）
    OptimizerConfig resolution_config = base_config;
    resolution_config.grid_resolution = 2;
    configs.push_back(resolution_config);
    
    // 启用节点回收
    OptimizerConfig recycling_config = base_config;
    recycling_config.use_node_recycling = true;
    configs.push_back(recycling_config);
    
    // 综合优化配置
    OptimizerConfig optimized_config;
    optimized_config.enable_parallel_search = true;
    optimized_config.use_improved_heuristic = true;
    optimized_config.max_iterations = 10000;
    optimized_config.goal_tolerance = 0.5;
    optimized_config.grid_resolution = 2;
    optimized_config.use_node_recycling = true;
    optimized_config.planning_timeout = 5.0;
    configs.push_back(optimized_config);
    
    // 对每个配置运行测试
    for (size_t i = 0; i < configs.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "测试A*配置 %zu/%zu", i + 1, configs.size());
        PerformanceMetrics metrics = runPerfTest("astar", configs[i]);
        
        // 保存结果
        results_.emplace_back(configs[i], metrics);
        
        // 输出结果
        RCLCPP_INFO(this->get_logger(), "  规划时间: %.4f 秒", metrics.planning_time);
        RCLCPP_INFO(this->get_logger(), "  路径长度: %.2f 米", metrics.total_distance);
        RCLCPP_INFO(this->get_logger(), "  扩展节点: %d", metrics.nodes_expanded);
        RCLCPP_INFO(this->get_logger(), "  内存使用: %.2f MB", metrics.memory_usage);
        RCLCPP_INFO(this->get_logger(), "  成功: %s", metrics.success ? "是" : "否");
        
        // 等待一秒，让系统稳定
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 分析结果，找出最佳配置
    int best_index = -1;
    double best_time = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < results_.size(); ++i) {
        if (results_[i].second.success && results_[i].second.planning_time < best_time) {
            best_time = results_[i].second.planning_time;
            best_index = i;
        }
    }
    
    if (best_index >= 0) {
        RCLCPP_INFO(this->get_logger(), "A*最佳配置:");
        RCLCPP_INFO(this->get_logger(), "  启用并行搜索: %s", 
                   results_[best_index].first.enable_parallel_search ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  使用改进启发式: %s", 
                   results_[best_index].first.use_improved_heuristic ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  网格分辨率: %d", 
                   results_[best_index].first.grid_resolution);
        RCLCPP_INFO(this->get_logger(), "  使用节点回收: %s", 
                   results_[best_index].first.use_node_recycling ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  规划时间: %.4f 秒", 
                   results_[best_index].second.planning_time);
    } else {
        RCLCPP_ERROR(this->get_logger(), "没有找到成功的A*配置");
    }
}

void PerformanceOptimizer::optimizeHybridAStar() {
    RCLCPP_INFO(this->get_logger(), "开始优化Hybrid A*算法...");
    
    // 创建不同的配置进行测试
    std::vector<OptimizerConfig> configs;
    
    // 基准配置
    OptimizerConfig base_config;
    base_config.enable_parallel_search = false;
    base_config.use_improved_heuristic = false;
    base_config.max_iterations = 10000;
    base_config.goal_tolerance = 0.5;
    base_config.grid_resolution = 1;
    base_config.use_node_recycling = false;
    base_config.planning_timeout = 5.0;
    configs.push_back(base_config);
    
    // 启用并行搜索
    OptimizerConfig parallel_config = base_config;
    parallel_config.enable_parallel_search = true;
    configs.push_back(parallel_config);
    
    // 使用改进的启发式函数
    OptimizerConfig heuristic_config = base_config;
    heuristic_config.use_improved_heuristic = true;
    configs.push_back(heuristic_config);
    
    // 降低网格分辨率（更粗糙但更快）
    OptimizerConfig resolution_config = base_config;
    resolution_config.grid_resolution = 2;
    configs.push_back(resolution_config);
    
    // 启用节点回收
    OptimizerConfig recycling_config = base_config;
    recycling_config.use_node_recycling = true;
    configs.push_back(recycling_config);
    
    // 调整目标容差
    OptimizerConfig tolerance_config = base_config;
    tolerance_config.goal_tolerance = 1.0;
    configs.push_back(tolerance_config);
    
    // 综合优化配置
    OptimizerConfig optimized_config;
    optimized_config.enable_parallel_search = true;
    optimized_config.use_improved_heuristic = true;
    optimized_config.max_iterations = 10000;
    optimized_config.goal_tolerance = 1.0;
    optimized_config.grid_resolution = 2;
    optimized_config.use_node_recycling = true;
    optimized_config.planning_timeout = 5.0;
    configs.push_back(optimized_config);
    
    // 对每个配置运行测试
    for (size_t i = 0; i < configs.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "测试Hybrid A*配置 %zu/%zu", i + 1, configs.size());
        PerformanceMetrics metrics = runPerfTest("hybrid_astar", configs[i]);
        
        // 保存结果
        results_.emplace_back(configs[i], metrics);
        
        // 输出结果
        RCLCPP_INFO(this->get_logger(), "  规划时间: %.4f 秒", metrics.planning_time);
        RCLCPP_INFO(this->get_logger(), "  路径长度: %.2f 米", metrics.total_distance);
        RCLCPP_INFO(this->get_logger(), "  扩展节点: %d", metrics.nodes_expanded);
        RCLCPP_INFO(this->get_logger(), "  内存使用: %.2f MB", metrics.memory_usage);
        RCLCPP_INFO(this->get_logger(), "  成功: %s", metrics.success ? "是" : "否");
        
        // 等待一秒，让系统稳定
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 分析结果，找出最佳配置
    int best_index = -1;
    double best_time = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < results_.size(); ++i) {
        if (results_[i].second.success && results_[i].second.planning_time < best_time) {
            best_time = results_[i].second.planning_time;
            best_index = i;
        }
    }
    
    if (best_index >= 0) {
        RCLCPP_INFO(this->get_logger(), "Hybrid A*最佳配置:");
        RCLCPP_INFO(this->get_logger(), "  启用并行搜索: %s", 
                   results_[best_index].first.enable_parallel_search ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  使用改进启发式: %s", 
                   results_[best_index].first.use_improved_heuristic ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  目标容差: %.2f", 
                   results_[best_index].first.goal_tolerance);
        RCLCPP_INFO(this->get_logger(), "  网格分辨率: %d", 
                   results_[best_index].first.grid_resolution);
        RCLCPP_INFO(this->get_logger(), "  使用节点回收: %s", 
                   results_[best_index].first.use_node_recycling ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  规划时间: %.4f 秒", 
                   results_[best_index].second.planning_time);
    } else {
        RCLCPP_ERROR(this->get_logger(), "没有找到成功的Hybrid A*配置");
    }
}

void PerformanceOptimizer::optimizeWithConfig(const std::string& planner_type, const OptimizerConfig& config) {
    RCLCPP_INFO(this->get_logger(), "使用自定义配置优化 %s ...", planner_type.c_str());
    
    PerformanceMetrics metrics = runPerfTest(planner_type, config);
    
    // 保存结果
    results_.emplace_back(config, metrics);
    
    // 输出结果
    RCLCPP_INFO(this->get_logger(), "  规划时间: %.4f 秒", metrics.planning_time);
    RCLCPP_INFO(this->get_logger(), "  路径长度: %.2f 米", metrics.total_distance);
    RCLCPP_INFO(this->get_logger(), "  扩展节点: %d", metrics.nodes_expanded);
    RCLCPP_INFO(this->get_logger(), "  内存使用: %.2f MB", metrics.memory_usage);
    RCLCPP_INFO(this->get_logger(), "  成功: %s", metrics.success ? "是" : "否");
}

PerformanceMetrics PerformanceOptimizer::runPerfTest(
    const std::string& planner_type, const OptimizerConfig& config) {
    
    // 重置结果标志
    std::lock_guard<std::mutex> lock(result_mutex_);
    received_result_ = false;
    
    // 创建测试场景
    createTestScenario(0);  // 使用固定场景以便比较
    
    // 发布地图
    map_pub_->publish(current_map_);
    
    // 创建规划请求
    auto_msgs::msg::PlanningRequest request;
    request.header.stamp = this->now();
    request.header.frame_id = "map";
    
    // 设置起点和终点
    request.start.header.frame_id = "map";
    request.start.pose.position.x = 5.0;
    request.start.pose.position.y = 5.0;
    request.start.pose.orientation.w = 1.0;
    
    request.goal.header.frame_id = "map";
    request.goal.pose.position.x = 15.0;
    request.goal.pose.position.y = 15.0;
    request.goal.pose.orientation.w = 1.0;
    
    // 设置规划器类型和参数
    request.planner_type = planner_type;
    request.consider_kinematic = (planner_type == "hybrid_astar");
    
    // 添加自定义参数（通过header中的frame_id传递）
    std::stringstream ss;
    ss << "config:";
    ss << "parallel=" << (config.enable_parallel_search ? "1" : "0") << ";";
    ss << "heuristic=" << (config.use_improved_heuristic ? "1" : "0") << ";";
    ss << "iterations=" << config.max_iterations << ";";
    ss << "tolerance=" << std::fixed << std::setprecision(2) << config.goal_tolerance << ";";
    ss << "resolution=" << config.grid_resolution << ";";
    ss << "recycling=" << (config.use_node_recycling ? "1" : "0") << ";";
    ss << "timeout=" << std::fixed << std::setprecision(1) << config.planning_timeout;
    
    request.header.frame_id = ss.str();
    
    // 记录初始内存使用
    double initial_memory = measureMemoryUsage();
    
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();
    
    // 发布规划请求
    request_pub_->publish(request);
    
    // 等待结果（使用配置中的超时时间）
    int timeout_seconds = static_cast<int>(std::ceil(config.planning_timeout));
    bool got_result = waitForResult(std::chrono::seconds(timeout_seconds));
    
    // 记录结束时间
    auto end_time = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(end_time - start_time).count();
    
    // 计算内存使用增量
    double final_memory = measureMemoryUsage();
    double memory_delta = final_memory - initial_memory;
    
    // 创建性能指标结果
    PerformanceMetrics metrics;
    metrics.planning_time = elapsed;
    metrics.memory_usage = memory_delta;
    metrics.success = got_result && !latest_path_.points.empty();
    metrics.nodes_expanded = 0;  // 这个值需要从规划器获取，目前先设为0
    
    if (metrics.success) {
        metrics.total_distance = latest_path_.total_distance;
        metrics.path_points = latest_path_.points.size();
    } else {
        metrics.total_distance = 0.0;
        metrics.path_points = 0;
    }
    
    return metrics;
}

void PerformanceOptimizer::saveOptimizationResults(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开文件保存优化结果: %s", filename.c_str());
        return;
    }
    
    // 写入CSV表头
    file << "规划器,并行搜索,改进启发式,最大迭代,目标容差,网格分辨率,节点回收,超时,规划时间,路径长度,扩展节点,路径点数,内存使用,成功\n";
    
    // 写入结果
    for (const auto& result : results_) {
        const auto& config = result.first;
        const auto& metrics = result.second;
        
        file << (result.first.enable_parallel_search ? "hybrid_astar" : "astar") << ","
             << (config.enable_parallel_search ? "是" : "否") << ","
             << (config.use_improved_heuristic ? "是" : "否") << ","
             << config.max_iterations << ","
             << config.goal_tolerance << ","
             << config.grid_resolution << ","
             << (config.use_node_recycling ? "是" : "否") << ","
             << config.planning_timeout << ","
             << metrics.planning_time << ","
             << metrics.total_distance << ","
             << metrics.nodes_expanded << ","
             << metrics.path_points << ","
             << metrics.memory_usage << ","
             << (metrics.success ? "是" : "否") << "\n";
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "优化结果已保存到: %s", filename.c_str());
}

void PerformanceOptimizer::applyOptimizedConfig(const std::string& planner_type, const OptimizerConfig& config) {
    // 在实际应用中，这里应该生成优化后的参数文件或直接更新ROS2参数服务器
    RCLCPP_INFO(this->get_logger(), "应用优化配置到 %s", planner_type.c_str());
    
    // 输出优化配置
    RCLCPP_INFO(this->get_logger(), "  启用并行搜索: %s", 
               config.enable_parallel_search ? "是" : "否");
    RCLCPP_INFO(this->get_logger(), "  使用改进启发式: %s", 
               config.use_improved_heuristic ? "是" : "否");
    RCLCPP_INFO(this->get_logger(), "  最大迭代次数: %d", 
               config.max_iterations);
    RCLCPP_INFO(this->get_logger(), "  目标容差: %.2f", 
               config.goal_tolerance);
    RCLCPP_INFO(this->get_logger(), "  网格分辨率: %d", 
               config.grid_resolution);
    RCLCPP_INFO(this->get_logger(), "  使用节点回收: %s", 
               config.use_node_recycling ? "是" : "否");
    RCLCPP_INFO(this->get_logger(), "  规划超时: %.1f秒", 
               config.planning_timeout);
    
    // 在这里可以创建一个参数文件
    std::string config_filename = planner_type + "_optimized_config.yaml";
    std::ofstream config_file(config_filename);
    if (config_file.is_open()) {
        config_file << "/**:\n";
        config_file << "  ros__parameters:\n";
        config_file << "    enable_parallel_search: " << (config.enable_parallel_search ? "true" : "false") << "\n";
        config_file << "    use_improved_heuristic: " << (config.use_improved_heuristic ? "true" : "false") << "\n";
        config_file << "    max_iterations: " << config.max_iterations << "\n";
        config_file << "    goal_tolerance: " << config.goal_tolerance << "\n";
        config_file << "    grid_resolution: " << config.grid_resolution << "\n";
        config_file << "    use_node_recycling: " << (config.use_node_recycling ? "true" : "false") << "\n";
        config_file << "    planning_timeout: " << config.planning_timeout << "\n";
        config_file.close();
        
        RCLCPP_INFO(this->get_logger(), "优化配置已保存到: %s", config_filename.c_str());
    }
}

void PerformanceOptimizer::createTestMap(int width, int height, double resolution) {
    auto_msgs::msg::GridMap map;
    map.header.frame_id = "map";
    map.header.stamp = this->now();
    map.width = width;
    map.height = height;
    map.resolution = resolution;
    map.data.resize(width * height, 0);  // 初始化为空闲
    
    // 设置地图原点
    map.origin.position.x = 0.0;
    map.origin.position.y = 0.0;
    map.origin.position.z = 0.0;
    map.origin.orientation.w = 1.0;
    
    current_map_ = map;
}

void PerformanceOptimizer::addRandomObstacles(int count) {
    if (current_map_.width == 0 || current_map_.height == 0) {
        RCLCPP_ERROR(this->get_logger(), "地图尚未初始化，无法添加障碍物");
        return;
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_x(0, current_map_.width - 1);
    std::uniform_int_distribution<> dis_y(0, current_map_.height - 1);
    
    for (int i = 0; i < count; ++i) {
        int x = dis_x(gen);
        int y = dis_y(gen);
        
        // 避免在起点和终点附近放置障碍物
        if ((x < 7 && y < 7) || (x > current_map_.width - 7 && y > current_map_.height - 7)) {
            continue;
        }
        
        // 添加障碍物
        int index = y * current_map_.width + x;
        if (index >= 0 && index < static_cast<int>(current_map_.data.size())) {
            current_map_.data[index] = 100;  // 设置为占用
        }
    }
}

void PerformanceOptimizer::createTestScenario(int scenario_id) {
    // 创建一个基本的测试地图
    createTestMap(20, 20, 1.0);
    
    switch (scenario_id) {
        case 0:  // 随机障碍物场景
            addRandomObstacles(30);
            break;
        case 1:  // 迷宫场景
            // 添加迷宫式障碍物
            for (int i = 0; i < 20; i += 4) {
                for (int j = 0; j < 16; ++j) {
                    int index = j * current_map_.width + i;
                    if (index >= 0 && index < static_cast<int>(current_map_.data.size())) {
                        current_map_.data[index] = 100;
                    }
                }
            }
            for (int i = 2; i < 20; i += 4) {
                for (int j = 4; j < 20; ++j) {
                    int index = j * current_map_.width + i;
                    if (index >= 0 && index < static_cast<int>(current_map_.data.size())) {
                        current_map_.data[index] = 100;
                    }
                }
            }
            break;
        case 2:  // 狭窄通道场景
            // 添加两堵墙，中间留一个狭窄通道
            for (int i = 0; i < 20; ++i) {
                for (int j = 8; j < 10; ++j) {
                    int index = i * current_map_.width + j;
                    if (i != 10) {  // 在中间留一个通道
                        current_map_.data[index] = 100;
                    }
                }
            }
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "未知场景ID: %d，使用随机障碍物", scenario_id);
            addRandomObstacles(30);
            break;
    }
}

void PerformanceOptimizer::planningPathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(result_mutex_);
    RCLCPP_INFO(this->get_logger(), "收到规划路径，包含 %zu 个点", msg->points.size());
    latest_path_ = *msg;
    received_result_ = true;
}

bool PerformanceOptimizer::waitForResult(std::chrono::seconds timeout) {
    auto start = std::chrono::steady_clock::now();
    
    while (rclcpp::ok()) {
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            if (received_result_) {
                return true;
            }
        }
        
        if (std::chrono::steady_clock::now() - start > timeout) {
            return false;  // 超时
        }
        
        // 非阻塞式等待
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return false;
}

double PerformanceOptimizer::measureMemoryUsage() {
    // 使用getrusage获取进程内存使用情况
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    
    // 返回兆字节单位的内存使用
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
}

} // namespace auto_integration_test

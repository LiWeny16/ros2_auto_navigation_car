#ifndef PERFORMANCE_OPTIMIZER_HPP_
#define PERFORMANCE_OPTIMIZER_HPP_

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <map>
#include <functional>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/planning_request.hpp"

namespace auto_integration_test {

// 性能优化器配置
struct OptimizerConfig {
    bool enable_parallel_search;   // 是否启用并行搜索
    bool use_improved_heuristic;   // 是否使用改进的启发式函数
    int max_iterations;            // 最大迭代次数
    double goal_tolerance;         // 目标容差
    int grid_resolution;           // 网格分辨率倍数
    bool use_node_recycling;       // 是否启用节点回收
    double planning_timeout;       // 规划超时时间（秒）
};

// 性能指标结构
struct PerformanceMetrics {
    double planning_time;          // 规划时间
    double total_distance;         // 路径总长度
    int nodes_expanded;            // 扩展的节点数量
    int path_points;               // 路径点数量
    double memory_usage;           // 内存使用量（MB）
    bool success;                  // 是否成功规划
};

// 性能优化器类
class PerformanceOptimizer : public rclcpp::Node {
public:
    PerformanceOptimizer();
    virtual ~PerformanceOptimizer() = default;
    
    // 使用默认配置优化A*算法
    void optimizeAStar();
    
    // 使用默认配置优化Hybrid A*算法
    void optimizeHybridAStar();
    
    // 使用自定义配置优化算法
    void optimizeWithConfig(const std::string& planner_type, const OptimizerConfig& config);
    
    // 运行性能测试
    PerformanceMetrics runPerfTest(const std::string& planner_type, const OptimizerConfig& config);
    
    // 保存优化结果
    void saveOptimizationResults(const std::string& filename);
    
    // 应用优化结果到实际系统
    void applyOptimizedConfig(const std::string& planner_type, const OptimizerConfig& config);

private:
    // 创建测试地图
    void createTestMap(int width, int height, double resolution);
    
    // 添加随机障碍物
    void addRandomObstacles(int count);
    
    // 创建测试场景
    void createTestScenario(int scenario_id);
    
    // 等待规划结果的回调
    void planningPathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg);
    
    // 等待超时函数
    bool waitForResult(std::chrono::seconds timeout);
    
    // 测量内存使用
    double measureMemoryUsage();
    
    // 当前地图
    auto_msgs::msg::GridMap current_map_;
    
    // 当前规划请求
    auto_msgs::msg::PlanningRequest current_request_;
    
    // 是否收到规划结果
    bool received_result_;
    
    // 最新规划路径
    auto_msgs::msg::PlanningPath latest_path_;
    
    // 规划性能结果
    std::vector<std::pair<OptimizerConfig, PerformanceMetrics>> results_;
    
    // 发布者和订阅者
    rclcpp::Publisher<auto_msgs::msg::GridMap>::SharedPtr map_pub_;
    rclcpp::Publisher<auto_msgs::msg::PlanningRequest>::SharedPtr request_pub_;
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_;
    
    // 互斥锁，用于处理回调
    std::mutex result_mutex_;
};

} // namespace auto_integration_test

#endif // PERFORMANCE_OPTIMIZER_HPP_

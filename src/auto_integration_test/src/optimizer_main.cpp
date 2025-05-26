#include "auto_integration_test/performance_optimizer.hpp"
#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::string results_file = "optimization_results.csv";
    if (argc > 1) {
        results_file = argv[1];
    }
    
    auto optimizer = std::make_shared<auto_integration_test::PerformanceOptimizer>();
    
    // 优化A*算法
    optimizer->optimizeAStar();
    
    // 优化Hybrid A*算法
    optimizer->optimizeHybridAStar();
    
    // 保存结果
    optimizer->saveOptimizationResults(results_file);
    
    // 自动应用优化后的配置
    auto_integration_test::OptimizerConfig optimized_astar_config;
    optimized_astar_config.enable_parallel_search = true;
    optimized_astar_config.use_improved_heuristic = true;
    optimized_astar_config.max_iterations = 10000;
    optimized_astar_config.goal_tolerance = 0.5;
    optimized_astar_config.grid_resolution = 2;
    optimized_astar_config.use_node_recycling = true;
    optimized_astar_config.planning_timeout = 5.0;
    
    auto_integration_test::OptimizerConfig optimized_hybrid_astar_config;
    optimized_hybrid_astar_config.enable_parallel_search = true;
    optimized_hybrid_astar_config.use_improved_heuristic = true;
    optimized_hybrid_astar_config.max_iterations = 10000;
    optimized_hybrid_astar_config.goal_tolerance = 1.0;
    optimized_hybrid_astar_config.grid_resolution = 2;
    optimized_hybrid_astar_config.use_node_recycling = true;
    optimized_hybrid_astar_config.planning_timeout = 5.0;
    
    optimizer->applyOptimizedConfig("astar", optimized_astar_config);
    optimizer->applyOptimizedConfig("hybrid_astar", optimized_hybrid_astar_config);
    
    rclcpp::shutdown();
    return 0;
}

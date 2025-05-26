#ifndef OPTIMIZED_A_STAR_PLANNER_HPP_
#define OPTIMIZED_A_STAR_PLANNER_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <functional>
#include <cmath>
#include <thread>
#include <mutex>
#include <future>
#include <atomic>

#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/path_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace auto_planning {

// 优化版A*节点结构
struct OptimizedNode {
    int x;
    int y;
    double g_cost;  // 从起点到当前节点的代价
    double h_cost;  // 从当前节点到目标的启发式估计代价
    double f_cost;  // f = g + h
    std::shared_ptr<OptimizedNode> parent;

    OptimizedNode(int x_, int y_, double g_, double h_, std::shared_ptr<OptimizedNode> parent_ = nullptr)
        : x(x_), y(y_), g_cost(g_), h_cost(h_), f_cost(g_ + h_), parent(parent_) {}
    
    // 用于优先队列排序
    bool operator>(const OptimizedNode& other) const {
        return f_cost > other.f_cost;
    }
};

// 用于哈希表的键
struct NodeKey {
    int x;
    int y;

    bool operator==(const NodeKey& other) const {
        return x == other.x && y == other.y;
    }
};

// 节点哈希函数
struct NodeKeyHash {
    std::size_t operator()(const NodeKey& key) const {
        return std::hash<int>()(key.x) ^ std::hash<int>()(key.y);
    }
};

// 优化配置
struct AStarConfig {
    bool enable_parallel_search = false;       // 启用并行搜索
    bool use_improved_heuristic = false;       // 使用改进的启发式函数
    int max_iterations = 10000;                // 最大迭代次数
    double goal_tolerance = 0.5;               // 目标容差
    int grid_resolution = 1;                   // 网格分辨率倍数
    bool use_node_recycling = false;           // 启用节点回收
    double planning_timeout = 5.0;             // 规划超时（秒）
};

// 优化版A*规划器
class OptimizedAStarPlanner {
public:
    OptimizedAStarPlanner();
    virtual ~OptimizedAStarPlanner() = default;

    // 使用A*算法计算路径
    auto_msgs::msg::PlanningPath plan(
        const auto_msgs::msg::GridMap& map,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const AStarConfig& config = AStarConfig());

protected:
    // 将世界坐标转换为网格坐标
    std::pair<int, int> worldToGrid(
        const auto_msgs::msg::GridMap& map, 
        const geometry_msgs::msg::Pose& pose,
        int resolution_multiplier);
    
    // 将网格坐标转换为世界坐标
    geometry_msgs::msg::Pose gridToWorld(
        const auto_msgs::msg::GridMap& map, 
        int x, int y,
        int resolution_multiplier);

    // 检查节点是否有效（在地图范围内且不是障碍物）
    bool isValidNode(const auto_msgs::msg::GridMap& map, int x, int y);

    // 标准启发式函数：使用欧几里得距离
    double heuristicEuclidean(int x1, int y1, int x2, int y2);
    
    // 改进的启发式函数：考虑曼哈顿距离和欧几里得距离
    double heuristicImproved(int x1, int y1, int x2, int y2, const auto_msgs::msg::GridMap& map);
    
    // 单线程搜索
    std::shared_ptr<OptimizedNode> searchSingleThreaded(
        const auto_msgs::msg::GridMap& map,
        const std::pair<int, int>& start_grid,
        const std::pair<int, int>& goal_grid,
        const AStarConfig& config);
    
    // 多线程搜索
    std::shared_ptr<OptimizedNode> searchMultiThreaded(
        const auto_msgs::msg::GridMap& map,
        const std::pair<int, int>& start_grid,
        const std::pair<int, int>& goal_grid,
        const AStarConfig& config);
    
    // 单向度搜索（按一个方向搜索一部分地图）
    std::shared_ptr<OptimizedNode> searchDirectional(
        const auto_msgs::msg::GridMap& map,
        const std::pair<int, int>& start_grid,
        const std::pair<int, int>& goal_grid,
        const AStarConfig& config,
        int direction_offset,
        int direction_count,
        std::atomic<bool>& found_flag);
    
    // 方向数组，用于8个方向的移动
    const std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},   // 上右下左
        {1, 1}, {1, -1}, {-1, -1}, {-1, 1}  // 对角线
    };
    
    // 节点对象池，用于减少内存分配开销
    class NodePool {
    public:
        NodePool(size_t initial_size = 1000) {
            nodes_.reserve(initial_size);
        }
        
        std::shared_ptr<OptimizedNode> createNode(int x, int y, double g, double h, 
                                              std::shared_ptr<OptimizedNode> parent = nullptr) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (use_recycling_ && !free_nodes_.empty()) {
                auto node = free_nodes_.back();
                free_nodes_.pop_back();
                node->x = x;
                node->y = y;
                node->g_cost = g;
                node->h_cost = h;
                node->f_cost = g + h;
                node->parent = parent;
                return node;
            } else {
                nodes_.push_back(std::make_shared<OptimizedNode>(x, y, g, h, parent));
                return nodes_.back();
            }
        }
        
        void recycleNode(std::shared_ptr<OptimizedNode> node) {
            if (use_recycling_) {
                std::lock_guard<std::mutex> lock(mutex_);
                free_nodes_.push_back(node);
            }
        }
        
        void setUseRecycling(bool use) {
            use_recycling_ = use;
        }
        
    private:
        std::vector<std::shared_ptr<OptimizedNode>> nodes_;
        std::vector<std::shared_ptr<OptimizedNode>> free_nodes_;
        std::mutex mutex_;
        bool use_recycling_ = false;
    };
    
    NodePool node_pool_;
};

} // namespace auto_planning

#endif // OPTIMIZED_A_STAR_PLANNER_HPP_

#ifndef HYBRID_A_STAR_PLANNER_HPP_
#define HYBRID_A_STAR_PLANNER_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <functional>
#include <cmath>
#include <limits>
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/path_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace auto_planning {

struct HybridNode {
    double x;        // x坐标
    double y;        // y坐标
    double theta;    // 方向角（弧度）
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到目标的启发式估计代价
    double f_cost;   // f = g + h
    double steering; // 转向角
    std::shared_ptr<HybridNode> parent;

    HybridNode(double x_, double y_, double theta_, double g_, double h_, 
               double steering_ = 0.0, std::shared_ptr<HybridNode> parent_ = nullptr)
        : x(x_), y(y_), theta(theta_), g_cost(g_), h_cost(h_), 
          f_cost(g_ + h_), steering(steering_), parent(parent_) {}
    
    // 用于优先队列排序
    bool operator>(const HybridNode& other) const {
        return f_cost > other.f_cost;
    }
};

// 用于哈希表的键
struct HybridNodeKey {
    int x_idx;
    int y_idx;
    int theta_idx;

    bool operator==(const HybridNodeKey& other) const {
        return x_idx == other.x_idx && y_idx == other.y_idx && theta_idx == other.theta_idx;
    }
};

// 哈希函数
struct HybridNodeKeyHash {
    std::size_t operator()(const HybridNodeKey& key) const {
        return std::hash<int>()(key.x_idx) ^ 
               std::hash<int>()(key.y_idx) ^ 
               std::hash<int>()(key.theta_idx);
    }
};

class HybridAStarPlanner {
public:
    HybridAStarPlanner(double wheel_base = 2.7);
    virtual ~HybridAStarPlanner() = default;

    // 使用Hybrid A*算法计算路径
    auto_msgs::msg::PlanningPath plan(
        const auto_msgs::msg::GridMap& map,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal);

protected:
    // 检查节点是否有效（在地图范围内且不是障碍物）
    bool isValidNode(const auto_msgs::msg::GridMap& map, double x, double y);

    // 启发式函数：使用欧几里得距离
    double heuristic(double x1, double y1, double x2, double y2);

    // 从世界坐标转换为网格坐标
    std::pair<int, int> worldToGrid(
        const auto_msgs::msg::GridMap& map, 
        double x, double y);
    
    // 从网格坐标转换为世界坐标
    std::pair<double, double> gridToWorld(
        const auto_msgs::msg::GridMap& map, 
        int x, int y);

    // 计算离散角度索引（将连续角度离散化）
    int thetaToIndex(double theta);
    
    // 从离散角度索引转换为角度
    double indexToTheta(int index);

    // 模拟车辆运动以生成下一个节点状态
    std::vector<HybridNode> getNextStates(
        const HybridNode& current, 
        const auto_msgs::msg::GridMap& map);

    // 轮距
    double wheel_base_;
    
    // 转向角度集合
    std::vector<double> steering_angles_ = {-0.6, -0.3, 0.0, 0.3, 0.6};
    
    // 移动距离步长
    double move_step_ = 0.1;
    
    // 方向角离散化参数
    const int angle_size_ = 72;  // 5度一个离散角度
};

} // namespace auto_planning

#endif // HYBRID_A_STAR_PLANNER_HPP_

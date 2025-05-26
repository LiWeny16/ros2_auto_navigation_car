#ifndef A_STAR_PLANNER_HPP_
#define A_STAR_PLANNER_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <functional>
#include <cmath>
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/path_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace auto_planning {

struct Node {
    int x;
    int y;
    double g_cost;  // 从起点到当前节点的代价
    double h_cost;  // 从当前节点到目标的启发式估计代价
    double f_cost;  // f = g + h
    std::shared_ptr<Node> parent;

    Node(int x_, int y_, double g_, double h_, std::shared_ptr<Node> parent_ = nullptr)
        : x(x_), y(y_), g_cost(g_), h_cost(h_), f_cost(g_ + h_), parent(parent_) {}
    
    // 用于优先队列排序
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

class AStarPlanner {
public:
    AStarPlanner();
    virtual ~AStarPlanner() = default;

    // 使用A*算法计算路径
    auto_msgs::msg::PlanningPath plan(
        const auto_msgs::msg::GridMap& map,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal);

protected:
    // 将世界坐标转换为网格坐标
    std::pair<int, int> worldToGrid(
        const auto_msgs::msg::GridMap& map, 
        const geometry_msgs::msg::Pose& pose);
    
    // 将网格坐标转换为世界坐标
    geometry_msgs::msg::Pose gridToWorld(
        const auto_msgs::msg::GridMap& map, 
        int x, int y);

    // 检查节点是否有效（在地图范围内且不是障碍物）
    bool isValidNode(const auto_msgs::msg::GridMap& map, int x, int y);

    // 启发式函数：使用欧几里得距离
    double heuristic(int x1, int y1, int x2, int y2);

    // 方向数组，用于8个方向的移动
    const std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},   // 上右下左
        {1, 1}, {1, -1}, {-1, -1}, {-1, 1}  // 对角线
    };
};

} // namespace auto_planning

#endif // A_STAR_PLANNER_HPP_

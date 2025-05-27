# auto_planning 模块文档

## 模块概述
auto_planning 是路径规划模块，实现多种路径规划算法，包括A*、Hybrid A*和优化版本，同时提供决策功能。

## 模块结构
```
auto_planning/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── path_planner_node.cpp           # 主规划节点
│   ├── decision_visualization_node.cpp # 决策可视化节点
│   ├── a_star_planner.cpp              # A*规划器
│   ├── hybrid_a_star_planner.cpp       # Hybrid A*规划器
│   ├── optimized_a_star_planner.cpp    # 优化A*规划器
│   └── decision_maker.cpp              # 决策模块
├── include/
│   └── auto_planning/
│       ├── path_planner_node.hpp
│       ├── a_star_planner.hpp
│       ├── hybrid_a_star_planner.hpp
│       ├── optimized_a_star_planner.hpp
│       └── decision_maker.hpp
├── test/
│   └── test_planner.cpp                # 单元测试
├── docs/                               # 模块文档
└── env-hooks/                          # 环境配置
```

## 主要功能

### 1. path_planner_node
**功能**: 路径规划的主节点，协调各种规划算法
**输入**: 
- `/planning_request` (auto_msgs/PlanningRequest): 规划请求
- `/grid_map` (auto_msgs/GridMap): 环境地图
- `/detected_objects` (visualization_msgs/MarkerArray): 障碍物信息
**输出**: 
- `/planning_path` (auto_msgs/PlanningPath): 规划路径
- `/planning_status` (std_msgs/String): 规划状态

### 2. 规划算法

#### A* Planner
**功能**: 基础A*路径规划算法
**特点**: 
- 保证最优解
- 适用于静态环境
- 计算效率高

**算法流程**:
```cpp
class AStarPlanner {
public:
    PlanningPath planPath(const PlanningRequest& request, const GridMap& map) {
        // 1. 初始化
        initializeSearch(request.start, request.goal);
        
        // 2. A*搜索
        while (!open_list_.empty()) {
            auto current = getLowestFCostNode();
            
            if (isGoalReached(current, request.goal)) {
                return reconstructPath(current);
            }
            
            expandNode(current, map);
        }
        
        return PlanningPath(); // 无解
    }
    
private:
    void initializeSearch(const Pose& start, const Pose& goal);
    Node* getLowestFCostNode();
    bool isGoalReached(const Node* node, const Pose& goal);
    void expandNode(Node* node, const GridMap& map);
    PlanningPath reconstructPath(const Node* goal_node);
    
    std::priority_queue<Node*> open_list_;
    std::unordered_set<Node*> closed_list_;
};
```

**启发式函数**:
```cpp
double AStarPlanner::calculateHeuristic(const Node* node, const Pose& goal) {
    // 欧几里得距离
    double dx = node->x - goal.position.x;
    double dy = node->y - goal.position.y;
    return std::sqrt(dx * dx + dy * dy);
}
```

#### Hybrid A* Planner
**功能**: 考虑车辆运动学约束的路径规划
**特点**:
- 考虑车辆转弯半径
- 生成平滑可执行路径
- 适用于车辆导航

**算法流程**:
```cpp
class HybridAStarPlanner {
public:
    PlanningPath planPath(const PlanningRequest& request, const GridMap& map) {
        // 1. 初始化状态空间 (x, y, θ)
        initializeStateSpace(request);
        
        // 2. Hybrid A*搜索
        while (!open_list_.empty()) {
            auto current = getLowestCostState();
            
            if (isGoalReached(current, request.goal)) {
                auto path = reconstructPath(current);
                return smoothPath(path);
            }
            
            expandState(current, map);
        }
        
        return PlanningPath();
    }
    
private:
    struct State {
        double x, y, theta;
        double g_cost, h_cost;
        State* parent;
    };
    
    void expandState(State* state, const GridMap& map);
    std::vector<State*> generateSuccessors(const State* state);
    bool isValidState(const State* state, const GridMap& map);
    PlanningPath smoothPath(const PlanningPath& raw_path);
};
```

**运动学模型**:
```cpp
std::vector<State*> HybridAStarPlanner::generateSuccessors(const State* state) {
    std::vector<State*> successors;
    
    // 车辆运动学参数
    double wheelbase = 2.5;  // 轴距
    double max_steering = M_PI / 4;  // 最大转向角
    
    // 生成不同转向角的后继状态
    for (double steering = -max_steering; steering <= max_steering; steering += 0.1) {
        double dt = 0.1;  // 时间步长
        double velocity = 1.0;  // 速度
        
        State* successor = new State();
        successor->x = state->x + velocity * dt * std::cos(state->theta);
        successor->y = state->y + velocity * dt * std::sin(state->theta);
        successor->theta = state->theta + velocity * dt * std::tan(steering) / wheelbase;
        
        successors.push_back(successor);
    }
    
    return successors;
}
```

#### Optimized A* Planner
**功能**: 性能优化的A*算法
**优化策略**:
- 并行计算
- 改进启发式函数
- 节点重用
- 自适应网格分辨率

```cpp
class OptimizedAStarPlanner {
public:
    PlanningPath planPath(const PlanningRequest& request, const GridMap& map) {
        // 1. 自适应分辨率
        auto adaptive_map = adaptResolution(map, request);
        
        // 2. 并行搜索
        return parallelSearch(request, adaptive_map);
    }
    
private:
    GridMap adaptResolution(const GridMap& map, const PlanningRequest& request);
    PlanningPath parallelSearch(const PlanningRequest& request, const GridMap& map);
    
    // 改进的启发式函数
    double improvedHeuristic(const Node* node, const Pose& goal, const GridMap& map);
    
    // 节点池管理
    NodePool node_pool_;
    
    // 并行处理
    ThreadPool thread_pool_;
};
```

### 3. decision_maker
**功能**: 高级决策模块
**输入**: 
- 当前车辆状态
- 环境信息
- 规划路径
**输出**: 
- 决策指令
- 行为规划

**决策类型**:
```cpp
enum class DecisionType {
    FOLLOW_PATH,        // 正常跟随路径
    AVOID_OBSTACLE,     // 障碍物避让
    EMERGENCY_STOP,     // 紧急停车
    REPLAN_PATH,        // 路径重规划
    WAIT,               // 等待
    REVERSE             // 倒车
};

class DecisionMaker {
public:
    DecisionType makeDecision(const VehicleState& state, 
                             const EnvironmentInfo& env,
                             const PlanningPath& path) {
        // 1. 安全检查
        if (hasImmediateDanger(state, env)) {
            return DecisionType::EMERGENCY_STOP;
        }
        
        // 2. 路径可行性检查
        if (!isPathValid(path, env)) {
            return DecisionType::REPLAN_PATH;
        }
        
        // 3. 障碍物检查
        if (hasObstacleOnPath(path, env)) {
            return DecisionType::AVOID_OBSTACLE;
        }
        
        // 4. 正常跟随
        return DecisionType::FOLLOW_PATH;
    }
    
private:
    bool hasImmediateDanger(const VehicleState& state, const EnvironmentInfo& env);
    bool isPathValid(const PlanningPath& path, const EnvironmentInfo& env);
    bool hasObstacleOnPath(const PlanningPath& path, const EnvironmentInfo& env);
};
```

## 节点接口

### path_planner_node
**订阅话题**:
- `/planning_request` (auto_msgs/PlanningRequest)
- `/grid_map` (auto_msgs/GridMap)
- `/detected_objects` (visualization_msgs/MarkerArray)

**发布话题**:
- `/planning_path` (auto_msgs/PlanningPath)
- `/planning_status` (std_msgs/String)
- `/planning_visualization` (visualization_msgs/MarkerArray)

**参数**:
- `planner_type`: 规划器类型 ("astar", "hybrid_astar", "optimized_astar")
- `max_planning_time`: 最大规划时间 (默认: 5.0s)
- `path_resolution`: 路径分辨率 (默认: 0.1m)
- `safety_margin`: 安全边距 (默认: 0.5m)

### decision_visualization_node
**订阅话题**:
- `/planning_path` (auto_msgs/PlanningPath)
- `/vehicle_state` (geometry_msgs/PoseStamped)

**发布话题**:
- `/decision_markers` (visualization_msgs/MarkerArray)

## 算法性能对比

| 算法 | 规划时间 | 内存使用 | 路径质量 | 适用场景 |
|------|----------|----------|----------|----------|
| A* | <1s | <100MB | 最优 | 静态环境 |
| Hybrid A* | <3s | <200MB | 平滑 | 车辆导航 |
| Optimized A* | <0.5s | <150MB | 近似最优 | 实时应用 |

## 测试方法

### 单独测试规划节点
```bash
# 启动规划节点
ros2 run auto_planning path_planner_node

# 发送测试规划请求
ros2 topic pub /planning_request auto_msgs/msg/PlanningRequest \
  "{header: {frame_id: 'map'}, 
    start: {pose: {position: {x: 0.0, y: 0.0}}}, 
    goal: {pose: {position: {x: 10.0, y: 10.0}}}, 
    planner_type: 'astar', 
    consider_kinematic: false}"

# 检查规划结果
ros2 topic echo /planning_path
```

### 算法性能测试
```bash
# 运行单元测试
colcon test --packages-select auto_planning

# 查看测试结果
colcon test-result --verbose

# 性能基准测试
ros2 run auto_planning benchmark_test
```

### 不同算法对比测试
```bash
# 测试A*算法
ros2 run auto_planning path_planner_node --ros-args -p planner_type:=astar

# 测试Hybrid A*算法
ros2 run auto_planning path_planner_node --ros-args -p planner_type:=hybrid_astar

# 测试优化A*算法
ros2 run auto_planning path_planner_node --ros-args -p planner_type:=optimized_astar
```

### 可视化测试
```bash
# 启动决策可视化
ros2 run auto_planning decision_visualization_node

# 在RViz中查看规划结果
rviz2 -d config/planning_visualization.rviz
```

## 参数调优

### A*算法参数
```yaml
astar:
  heuristic_weight: 1.0      # 启发式权重
  diagonal_cost: 1.414       # 对角线移动代价
  straight_cost: 1.0         # 直线移动代价
  max_iterations: 10000      # 最大迭代次数
```

### Hybrid A*参数
```yaml
hybrid_astar:
  wheelbase: 2.5             # 车辆轴距
  max_steering_angle: 0.785  # 最大转向角
  step_size: 0.1             # 步长
  angle_resolution: 0.1      # 角度分辨率
  reverse_penalty: 2.0       # 倒车惩罚
```

### 优化A*参数
```yaml
optimized_astar:
  thread_count: 4            # 线程数
  node_pool_size: 100000     # 节点池大小
  adaptive_resolution: true  # 自适应分辨率
  memory_limit: 500          # 内存限制(MB)
```

## 性能指标
- **A*算法**: 规划时间 <1s, 内存使用 <100MB
- **Hybrid A***: 规划时间 <3s, 路径平滑度 >90%
- **优化A***: 规划时间 <0.5s, 并行效率 >80%

## 故障排除

### 常见问题
1. **规划失败**
   - 检查起点和终点是否在可行区域
   - 验证地图数据完整性
   - 调整安全边距参数

2. **规划时间过长**
   - 减少搜索空间
   - 使用优化算法
   - 调整超时参数

3. **路径质量差**
   - 调整启发式权重
   - 使用路径平滑算法
   - 检查地图分辨率

### 调试工具
```bash
# 查看规划统计
ros2 topic echo /planning_status

# 可视化搜索过程
ros2 run auto_planning search_visualizer

# 性能分析
ros2 run auto_planning profiler
```

## 依赖关系
- **输入依赖**: auto_msgs, auto_simulation, auto_perception
- **输出依赖**: auto_control (提供规划路径)
- **外部依赖**: 
  - Eigen3
  - OMPL (可选)
  - OpenMP (并行计算)

## 扩展功能
- 多目标路径规划
- 动态环境适应
- 学习型规划器
- 多车协同规划
- 语义路径规划 
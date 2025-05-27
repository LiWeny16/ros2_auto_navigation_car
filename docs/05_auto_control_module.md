# auto_control 模块文档

## 模块概述
auto_control 是控制模块，实现纯跟踪控制器，根据规划路径生成车辆控制指令。

## 模块结构
```
auto_control/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── controller_node.cpp          # 主控制节点
│   └── pure_pursuit_controller.cpp  # 纯跟踪控制器
├── include/
│   └── auto_control/
│       ├── controller_node.hpp
│       └── pure_pursuit_controller.hpp
├── test/
│   └── test_controller.cpp          # 单元测试
└── rviz/                            # RViz配置
    └── control_visualization.rviz
```

## 主要功能

### 1. controller_node
**功能**: 控制系统的主节点，实现车辆运动控制
**输入**: 
- `/planning_path` (auto_msgs/PlanningPath): 规划路径
- `/vehicle_state` (geometry_msgs/PoseStamped): 车辆当前状态
**输出**: 
- `/control_cmd` (geometry_msgs/Twist): 控制指令
- `/vehicle_trajectory` (nav_msgs/Path): 车辆轨迹

### 2. pure_pursuit_controller
**功能**: 纯跟踪控制算法实现
**特点**:
- 几何路径跟踪
- 平滑转向控制
- 速度自适应调节

## 控制算法详解

### 纯跟踪算法原理
纯跟踪算法是一种几何路径跟踪方法，通过在路径上选择前瞻点，计算车辆到前瞻点的转向角。

```cpp
class PurePursuitController {
public:
    ControlCommand calculateControl(const VehicleState& current_state,
                                   const PlanningPath& path) {
        // 1. 找到前瞻点
        auto lookahead_point = findLookaheadPoint(current_state, path);
        
        // 2. 计算转向角
        double steering_angle = calculateSteeringAngle(current_state, lookahead_point);
        
        // 3. 计算目标速度
        double target_velocity = calculateTargetVelocity(current_state, path);
        
        // 4. 生成控制指令
        ControlCommand cmd;
        cmd.steering_angle = steering_angle;
        cmd.velocity = target_velocity;
        
        return cmd;
    }
    
private:
    Point findLookaheadPoint(const VehicleState& state, const PlanningPath& path);
    double calculateSteeringAngle(const VehicleState& state, const Point& lookahead);
    double calculateTargetVelocity(const VehicleState& state, const PlanningPath& path);
};
```

### 数学模型

#### 1. 前瞻点选择
前瞻距离根据当前速度动态调整：
```cpp
double PurePursuitController::calculateLookaheadDistance(double current_velocity) {
    // ld = k * v + ld_min
    double lookahead_distance = lookahead_gain_ * current_velocity + min_lookahead_distance_;
    
    // 限制在合理范围内
    lookahead_distance = std::clamp(lookahead_distance, 
                                   min_lookahead_distance_, 
                                   max_lookahead_distance_);
    
    return lookahead_distance;
}
```

#### 2. 转向角计算
基于几何关系计算前轮转向角：
```cpp
double PurePursuitController::calculateSteeringAngle(const VehicleState& state, 
                                                    const Point& lookahead_point) {
    // 计算车辆到前瞻点的向量
    double dx = lookahead_point.x - state.position.x;
    double dy = lookahead_point.y - state.position.y;
    
    // 转换到车辆坐标系
    double cos_yaw = std::cos(state.yaw);
    double sin_yaw = std::sin(state.yaw);
    
    double local_x = dx * cos_yaw + dy * sin_yaw;
    double local_y = -dx * sin_yaw + dy * cos_yaw;
    
    // 计算前瞻距离
    double lookahead_distance = std::sqrt(local_x * local_x + local_y * local_y);
    
    // 纯跟踪公式: δ = arctan(2 * L * sin(α) / ld)
    double alpha = std::atan2(local_y, local_x);
    double steering_angle = std::atan2(2.0 * wheelbase_ * std::sin(alpha), lookahead_distance);
    
    // 限制转向角
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
    
    return steering_angle;
}
```

#### 3. 速度控制
根据路径曲率和距离调整速度：
```cpp
double PurePursuitController::calculateTargetVelocity(const VehicleState& state, 
                                                     const PlanningPath& path) {
    // 1. 基于曲率的速度限制
    double curvature = getCurrentPathCurvature(state, path);
    double curvature_velocity = std::sqrt(max_lateral_acceleration_ / std::max(std::abs(curvature), min_curvature_));
    
    // 2. 基于距离的速度控制
    double distance_to_goal = calculateDistanceToGoal(state, path);
    double distance_velocity = calculateDecelerationVelocity(distance_to_goal);
    
    // 3. 基于障碍物的速度限制
    double obstacle_velocity = calculateObstacleVelocity(state, path);
    
    // 4. 取最小值作为目标速度
    double target_velocity = std::min({max_velocity_, 
                                      curvature_velocity, 
                                      distance_velocity, 
                                      obstacle_velocity});
    
    return std::max(target_velocity, min_velocity_);
}
```

### 高级控制特性

#### 1. 自适应前瞻距离
```cpp
double PurePursuitController::adaptiveLookaheadDistance(const VehicleState& state, 
                                                       const PlanningPath& path) {
    // 基础前瞻距离
    double base_distance = lookahead_gain_ * state.velocity + min_lookahead_distance_;
    
    // 根据路径曲率调整
    double curvature = getCurrentPathCurvature(state, path);
    double curvature_factor = 1.0 / (1.0 + curvature_weight_ * std::abs(curvature));
    
    // 根据横向误差调整
    double lateral_error = calculateLateralError(state, path);
    double error_factor = 1.0 + error_weight_ * std::abs(lateral_error);
    
    return base_distance * curvature_factor * error_factor;
}
```

#### 2. 平滑控制输出
```cpp
ControlCommand PurePursuitController::smoothControl(const ControlCommand& raw_cmd) {
    ControlCommand smoothed_cmd = raw_cmd;
    
    // 转向角平滑
    double steering_rate = (raw_cmd.steering_angle - previous_steering_) / control_dt_;
    if (std::abs(steering_rate) > max_steering_rate_) {
        double sign = (steering_rate > 0) ? 1.0 : -1.0;
        smoothed_cmd.steering_angle = previous_steering_ + sign * max_steering_rate_ * control_dt_;
    }
    
    // 速度平滑
    double acceleration = (raw_cmd.velocity - previous_velocity_) / control_dt_;
    if (acceleration > max_acceleration_) {
        smoothed_cmd.velocity = previous_velocity_ + max_acceleration_ * control_dt_;
    } else if (acceleration < -max_deceleration_) {
        smoothed_cmd.velocity = previous_velocity_ - max_deceleration_ * control_dt_;
    }
    
    // 更新历史值
    previous_steering_ = smoothed_cmd.steering_angle;
    previous_velocity_ = smoothed_cmd.velocity;
    
    return smoothed_cmd;
}
```

## 节点接口

### controller_node
**订阅话题**:
- `/planning_path` (auto_msgs/PlanningPath): 目标路径
- `/vehicle_state` (geometry_msgs/PoseStamped): 车辆状态

**发布话题**:
- `/control_cmd` (geometry_msgs/Twist): 控制指令
- `/vehicle_trajectory` (nav_msgs/Path): 实际轨迹
- `/control_status` (std_msgs/String): 控制状态
- `/control_debug` (auto_msgs/ControlDebug): 调试信息

**参数**:
- `lookahead_distance`: 前瞻距离 (默认: 2.0m)
- `lookahead_gain`: 前瞻增益 (默认: 0.5)
- `max_velocity`: 最大速度 (默认: 2.0m/s)
- `max_acceleration`: 最大加速度 (默认: 1.0m/s²)
- `max_deceleration`: 最大减速度 (默认: 2.0m/s²)
- `wheelbase`: 车辆轴距 (默认: 2.5m)
- `control_frequency`: 控制频率 (默认: 20Hz)

## 控制性能评估

### 评估指标
```cpp
struct ControlMetrics {
    double lateral_error_mean;      // 平均横向误差
    double lateral_error_max;       // 最大横向误差
    double lateral_error_std;       // 横向误差标准差
    
    double heading_error_mean;      // 平均航向误差
    double heading_error_max;       // 最大航向误差
    
    double velocity_tracking_error; // 速度跟踪误差
    double control_smoothness;      // 控制平滑度
    
    double computation_time;        // 计算时间
    double control_frequency_actual; // 实际控制频率
};
```

### 性能监控
```cpp
class ControlPerformanceMonitor {
public:
    void updateMetrics(const VehicleState& state, 
                      const PlanningPath& path,
                      const ControlCommand& cmd) {
        // 计算横向误差
        double lateral_error = calculateLateralError(state, path);
        lateral_errors_.push_back(lateral_error);
        
        // 计算航向误差
        double heading_error = calculateHeadingError(state, path);
        heading_errors_.push_back(heading_error);
        
        // 记录控制指令
        control_commands_.push_back(cmd);
        
        // 更新统计信息
        updateStatistics();
    }
    
    ControlMetrics getMetrics() const {
        return current_metrics_;
    }
    
private:
    std::vector<double> lateral_errors_;
    std::vector<double> heading_errors_;
    std::vector<ControlCommand> control_commands_;
    ControlMetrics current_metrics_;
    
    void updateStatistics();
};
```

## 测试方法

### 单独测试控制节点
```bash
# 启动控制节点
ros2 run auto_control controller_node

# 发布测试路径
ros2 topic pub /planning_path auto_msgs/msg/PlanningPath \
  "{header: {frame_id: 'map'}, 
    points: [{pose: {pose: {position: {x: 0.0, y: 0.0}}}, velocity: 1.0}], 
    total_length: 10.0, 
    planner_type: 'test'}"

# 发布车辆状态
ros2 topic pub /vehicle_state geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, 
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, 
           orientation: {w: 1.0}}}"

# 检查控制指令
ros2 topic echo /control_cmd
```

### 控制性能测试
```bash
# 运行单元测试
colcon test --packages-select auto_control

# 测试控制精度
ros2 run auto_control test_tracking_accuracy

# 测试控制稳定性
ros2 run auto_control test_stability

# 监控控制频率
ros2 topic hz /control_cmd
```

### 参数调优测试
```bash
# 测试不同前瞻距离
ros2 run auto_control controller_node --ros-args -p lookahead_distance:=1.5

# 测试不同最大速度
ros2 run auto_control controller_node --ros-args -p max_velocity:=1.5

# 实时调整参数
ros2 param set /controller_node lookahead_distance 2.5
ros2 param set /controller_node max_velocity 2.5
```

### 可视化测试
```bash
# 启动RViz查看控制效果
rviz2 -d rviz/control_visualization.rviz

# 查看车辆轨迹
ros2 topic echo /vehicle_trajectory

# 监控控制状态
ros2 topic echo /control_status

# 查看调试信息
ros2 topic echo /control_debug
```

## 参数调优指南

### 基础参数调优
```yaml
# control_params.yaml
controller:
  # 前瞻参数
  lookahead_distance: 2.0      # 基础前瞻距离
  lookahead_gain: 0.5          # 速度相关增益
  min_lookahead_distance: 1.0  # 最小前瞻距离
  max_lookahead_distance: 5.0  # 最大前瞻距离
  
  # 速度参数
  max_velocity: 2.0            # 最大速度
  min_velocity: 0.1            # 最小速度
  max_acceleration: 1.0        # 最大加速度
  max_deceleration: 2.0        # 最大减速度
  
  # 转向参数
  max_steering_angle: 0.785    # 最大转向角 (45度)
  max_steering_rate: 1.0       # 最大转向角速度
  
  # 车辆参数
  wheelbase: 2.5               # 轴距
  max_lateral_acceleration: 2.0 # 最大横向加速度
  
  # 控制参数
  control_frequency: 20.0      # 控制频率
```

### 高级参数调优
```yaml
advanced_controller:
  # 自适应参数
  adaptive_lookahead: true
  curvature_weight: 1.0
  error_weight: 0.5
  
  # 平滑参数
  enable_smoothing: true
  steering_filter_gain: 0.8
  velocity_filter_gain: 0.9
  
  # 性能参数
  lateral_error_threshold: 0.2
  heading_error_threshold: 0.1
  velocity_error_threshold: 0.5
```

## 性能指标
- **跟踪精度**: 横向误差 <0.1m
- **控制延迟**: <50ms
- **稳定性**: 无振荡
- **收敛时间**: <2s
- **控制频率**: 20Hz

## 安全特性

### 安全检查
```cpp
class SafetyChecker {
public:
    bool isSafeCommand(const ControlCommand& cmd, const VehicleState& state) {
        // 1. 速度安全检查
        if (cmd.velocity > max_safe_velocity_ || cmd.velocity < 0) {
            return false;
        }
        
        // 2. 转向角安全检查
        if (std::abs(cmd.steering_angle) > max_safe_steering_) {
            return false;
        }
        
        // 3. 加速度安全检查
        double acceleration = (cmd.velocity - state.velocity) / control_dt_;
        if (std::abs(acceleration) > max_safe_acceleration_) {
            return false;
        }
        
        // 4. 转向角速度安全检查
        double steering_rate = (cmd.steering_angle - previous_steering_) / control_dt_;
        if (std::abs(steering_rate) > max_safe_steering_rate_) {
            return false;
        }
        
        return true;
    }
    
private:
    double max_safe_velocity_ = 3.0;
    double max_safe_steering_ = M_PI / 3;  // 60度
    double max_safe_acceleration_ = 2.0;
    double max_safe_steering_rate_ = 2.0;
};
```

### 紧急停车
```cpp
ControlCommand PurePursuitController::emergencyStop() {
    ControlCommand emergency_cmd;
    emergency_cmd.velocity = 0.0;
    emergency_cmd.steering_angle = 0.0;
    emergency_cmd.emergency_brake = true;
    
    return emergency_cmd;
}
```

## 故障排除

### 常见问题
1. **跟踪精度差**
   - 调整前瞻距离
   - 检查路径质量
   - 优化控制参数

2. **控制振荡**
   - 增加控制平滑
   - 降低控制增益
   - 检查传感器噪声

3. **响应延迟**
   - 提高控制频率
   - 优化算法效率
   - 检查系统负载

### 调试工具
```bash
# 查看控制统计
ros2 topic echo /control_status

# 可视化控制过程
ros2 run auto_control control_visualizer

# 性能分析
ros2 run auto_control control_profiler

# 参数调优助手
ros2 run auto_control parameter_tuner
```

## 依赖关系
- **输入依赖**: auto_msgs, auto_planning
- **输出依赖**: auto_simulation (接收控制指令)
- **外部依赖**: 
  - Eigen3 (数学计算)
  - 车辆运动学模型

## 扩展功能
- 模型预测控制 (MPC)
- 自适应控制
- 鲁棒控制
- 学习型控制器
- 多车协调控制 
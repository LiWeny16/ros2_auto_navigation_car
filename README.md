# 自动驾驶仿真系统文档

## 系统概述

这个自动驾驶仿真系统基于ROS2 Humble实现，提供了一个完整的自动驾驶软件栈的仿真环境，包括感知、规划、决策和控制等关键模块，可以在不需要实际硬件的情况下进行自动驾驶算法的开发与测试。

系统采用了模块化设计，具有良好的扩展性和可维护性，同时支持使用FastDDS进行中间件通信和MQTT进行与外部系统的消息传递。

## 系统架构

系统由以下几个主要部分组成：

1. **仿真模块 (auto_simulation)**
   - 负责生成模拟环境（地图和障碍物）
   - 发送规划请求
   - 可视化仿真结果
   - 提供MQTT桥接与外部系统通信

2. **感知模块 (auto_perception)**
   - 从网格地图中检测并跟踪障碍物
   - 生成障碍物的三维表示和分类

3. **规划模块 (auto_planning)**
   - 实现A*和Hybrid A*路径规划算法
   - 提供决策模块，处理复杂场景
   - 生成可执行的路径

4. **控制模块 (auto_control)**
   - 实现纯跟踪控制器
   - 根据规划路径生成控制指令
   - 模拟车辆运动

5. **消息模块 (auto_msgs)**
   - 定义系统中使用的所有消息类型

## 功能特性

- **多种路径规划算法**：支持A*和Hybrid A*算法，以及针对性能优化的改进算法
- **完整感知功能**：从地图数据中自动检测和识别障碍物
- **决策系统**：基于当前环境和车辆状态做出驾驶决策
- **平滑控制**：使用纯跟踪控制器实现车辆平稳控制
- **可视化界面**：通过RViz提供直观的可视化界面
- **MQTT通信**：支持与外部系统通过MQTT协议进行通信
- **FastDDS支持**：使用高性能的FastDDS中间件进行内部通信
- **性能优化**：提供性能分析和优化工具，自动调整算法参数
- **集成测试**：完整的系统集成测试验证功能正确性
- **错误处理**：全面的错误处理和诊断机制

## 安装指南

### 系统要求

- Ubuntu 22.04
- ROS2 Humble
- FastDDS
- MQTT (Mosquitto)

### 安装步骤

1. 安装ROS2 Humble（如果尚未安装）:
   ```bash
   sudo apt update && sudo apt install -y ros-humble-desktop
   ```

2. 使用提供的安装脚本安装依赖:
   ```bash
   cd /home/onion/samples/autonomous_driving_ws
   chmod +x install_dependencies.sh
   ./install_dependencies.sh
   ```

3. 编译工作空间:
   ```bash
   cd /home/onion/samples/autonomous_driving_ws
   colcon build --symlink-install
   ```

4. 加载环境:
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/onion/samples/autonomous_driving_ws/install/setup.bash
   ```

## 使用指南

### 启动系统

通过launch文件启动完整系统:

```bash
ros2 launch auto_simulation auto_driving.launch.xml
```

这将启动所有必要的节点，包括:
- 仿真节点
- 感知节点
- 规划节点
- 控制节点
- 决策可视化节点
- MQTT桥接节点
- RViz可视化工具

### 修改配置

可以通过传递参数修改系统配置:

```bash
# 使用A*规划器
ros2 launch auto_simulation auto_driving.launch.xml planner_type:=astar

# 不启动RViz
ros2 launch auto_simulation auto_driving.launch.xml use_rviz:=false
```

### 通过ROS2接口与系统交互

可以使用ROS2命令行工具与系统交互:

```bash
# 查看所有话题
ros2 topic list

# 发送规划请求
ros2 topic pub --once /planning_request auto_msgs/msg/PlanningRequest "..."

# 查看当前路径
ros2 topic echo /planning_path
```

### 通过MQTT与系统交互

系统提供MQTT接口与外部系统交互:

```bash
# 订阅规划路径
mosquitto_sub -t "auto_driving/planning_path"

# 请求规划
mosquitto_pub -t "auto_driving/command/request_planning" -m "..."
```

## 包结构说明

### auto_msgs 包

包含系统使用的自定义消息定义：
- `GridMap.msg`: 表示网格地图
- `PathPoint.msg`: 表示路径中的单个点
- `PlanningPath.msg`: 表示完整的规划路径
- `PlanningRequest.msg`: 表示路径规划请求

### auto_planning 包

包含路径规划和决策相关的实现：
- `a_star_planner`: A*路径规划算法
- `hybrid_a_star_planner`: Hybrid A*路径规划算法
- `decision_maker`: 决策模块
- `path_planner_node`: 主规划节点
- `decision_visualization_node`: 决策可视化节点

### auto_perception 包

包含感知相关的实现：
- `object_detector`: 障碍物检测器
- `perception_node`: 主感知节点

### auto_control 包

包含控制相关的实现：
- `pure_pursuit_controller`: 纯跟踪控制器
- `controller_node`: 主控制节点

### auto_simulation 包

包含仿真环境相关的实现：
- `simulation_node`: 仿真节点
- `mqtt_bridge`: MQTT通信桥接
- `mqtt_bridge_node`: MQTT桥接节点

## 测试

系统包含单元测试和集成测试以确保各组件的正确性：

```bash
# 运行规划模块测试
colcon test --packages-select auto_planning

# 运行控制模块测试
colcon test --packages-select auto_control

# 运行集成测试和性能优化
./run_tests_and_optimize.sh
```

### 集成测试

集成测试验证整个系统的工作流程，测试包括：

- 基本路径规划（A*和Hybrid A*）
- 障碍物避让
- 紧急停车
- 路径重规划
- 控制精度

测试结果保存在`integration_test_results.csv`文件中。

### 性能优化

性能优化工具可以帮助调整规划算法以获得最佳性能：

- 测试不同的优化配置
- 并行计算
- 改进的启发式函数
- 节点回收利用
- 网格分辨率自适应

优化结果保存在`performance_optimization_results.csv`文件中，最佳配置自动应用于系统。

## 开发指南

### 添加新的规划算法

1. 在`auto_planning/include/auto_planning`目录下创建新算法的头文件
2. 在`auto_planning/src`目录下实现算法
3. 在`path_planner_node.cpp`中添加对新算法的支持
4. 更新`CMakeLists.txt`添加新文件

### 添加新的控制算法

1. 在`auto_control/include/auto_control`目录下创建新控制器的头文件
2. 在`auto_control/src`目录下实现控制器
3. 在`controller_node.cpp`中添加对新控制器的支持
4. 更新`CMakeLists.txt`添加新文件

## 常见问题与解决

1. **Q: 系统启动后没有生成地图怎么办？**
   A: 确保`simulation_node`正常运行，检查ROS2日志，可能需要重启节点。

2. **Q: 路径规划失败怎么办？**
   A: 检查起点和终点是否在可行区域内，检查地图是否包含障碍物阻断了所有可能的路径。

3. **Q: MQTT连接失败怎么办？**
   A: 确保Mosquitto服务正在运行，检查连接参数是否正确，可能需要安装Mosquitto客户端和服务器。

4. **Q: 如何调整车辆运动参数？**
   A: 在`controller_node`启动参数中调整`lookahead_distance`和`max_velocity`等参数。

## 许可证

本系统采用MIT许可证。

## 贡献指南

欢迎贡献代码改进系统功能。请遵循以下步骤：
1. Fork仓库
2. 创建功能分支
3. 提交更改
4. 推送到分支
5. 创建Pull Request

## 联系方式

如有问题，请通过Issue系统提交。
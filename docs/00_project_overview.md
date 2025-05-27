# 自动驾驶仿真系统架构文档

## 项目概述

这是一个基于ROS2 Humble的完整自动驾驶仿真系统，采用模块化设计，提供了从感知、规划到控制的完整软件栈。系统支持多种路径规划算法，具有良好的可扩展性和可维护性。

## 系统架构图

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   仿真模块      │───▶│   感知模块      │───▶│   规划模块      │
│ auto_simulation │    │ auto_perception │    │ auto_planning   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
    地图数据                 障碍物信息               规划路径
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 ▼
                    ┌─────────────────┐
                    │   控制模块      │
                    │  auto_control   │
                    └─────────────────┘
                             │
                             ▼
                        控制指令
```

## 模块组成

### 核心模块
1. **auto_msgs** - 消息定义模块
2. **auto_simulation** - 仿真环境模块  
3. **auto_perception** - 感知模块
4. **auto_planning** - 路径规划模块
5. **auto_control** - 控制模块
6. **auto_integration_test** - 集成测试模块

### 数据流向
```
仿真模块 → 感知模块 → 规划模块 → 控制模块
    ↓         ↓         ↓         ↓
  地图数据   障碍物    规划路径   控制指令
```

## 主要特性

- **多种规划算法**: A*、Hybrid A*、优化A*
- **完整感知功能**: 障碍物检测和跟踪
- **平滑控制**: 纯跟踪控制器
- **可视化支持**: RViz集成
- **外部通信**: MQTT桥接
- **性能优化**: 并行计算和算法优化
- **完整测试**: 单元测试和集成测试

## 快速开始

### 1. 安装依赖
```bash
./install_dependencies.sh
```

### 2. 编译系统
```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 启动系统
```bash
ros2 launch auto_simulation auto_driving.launch.xml
```

### 4. 运行测试
```bash
./run_tests_and_optimize.sh
```

## 文档结构

- `00_project_overview.md` - 项目总览（本文档）
- `01_auto_msgs_module.md` - 消息定义模块
- `02_auto_simulation_module.md` - 仿真模块
- `03_auto_perception_module.md` - 感知模块
- `04_auto_planning_module.md` - 规划模块
- `05_auto_control_module.md` - 控制模块
- `06_auto_integration_test_module.md` - 集成测试模块
- `07_system_testing_guide.md` - 系统测试指南
- `08_development_guide.md` - 开发指南
- `09_troubleshooting.md` - 故障排除指南

## 技术栈

- **操作系统**: Ubuntu 22.04
- **中间件**: ROS2 Humble
- **通信**: FastDDS + MQTT
- **可视化**: RViz2
- **编程语言**: C++17
- **构建系统**: CMake + Colcon

## 性能指标

- **规划时间**: <1s (A*), <3s (Hybrid A*)
- **控制精度**: 横向误差 <0.1m
- **系统延迟**: <100ms
- **内存使用**: <500MB
- **CPU使用**: <30%

## 许可证

GNU3.0 License

## 贡献指南

欢迎提交Issue和Pull Request来改进系统功能。 
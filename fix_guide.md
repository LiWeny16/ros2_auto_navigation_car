# 自动驾驶项目问题修复指南

经过全面分析，项目存在以下几类主要问题：

## 1. CMake 配置问题

- `auto_planning/CMakeLists.txt`中存在重复的`ament_package()`调用
- 环境钩子(environment hooks)顺序问题：必须在`ament_package()`前调用`ament_environment_hooks()`

## 2. tf2相关问题

在多个文件中，使用了已弃用的 TF2 API：

```cpp
// 旧写法 - 已弃用
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
double yaw = tf2::getYaw(orientation);

// 新写法 - 推荐
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
double yaw = tf2::impl::getYaw(orientation);
```

需要修改以下文件：
- auto_control/src/controller_node.cpp
- auto_planning/src/decision_visualization_node.cpp
- auto_planning/test/test_planner.cpp
- auto_control/test/test_controller.cpp

## 3. 缺少头文件包含

在多个测试文件中缺少必要的头文件，如：
- `<cmath>` - 提供数学函数如 std::sin, std::cos
- 正确的 tf2 头文件

## 4. 依赖问题

- `auto_planning` 依赖 `auto_perception`
- 需要确保先编译 `auto_perception` 并安装其头文件

## 修复步骤

1. 确保 `auto_msgs` 能够正确编译
2. 修复 `auto_perception` 中的问题
3. 修复所有 TF2 相关的 API 调用
4. 清理 CMake 配置文件中的错误
5. 加入正确的头文件包含

## 自动驾驶系统构建顺序

要正确构建系统，按照以下顺序：

```bash
source /opt/ros/humble/setup.bash

# 1. 先编译消息
colcon build --packages-select auto_msgs

# 2. 编译感知模块
colcon build --packages-select auto_perception

# 3. 编译规划模块
colcon build --packages-select auto_planning

# 4. 编译控制模块
colcon build --packages-select auto_control

# 5. 编译仿真模块
colcon build --packages-select auto_simulation

# 6. 编译集成测试
colcon build --packages-select auto_integration_test
```

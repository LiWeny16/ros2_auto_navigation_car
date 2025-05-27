# 系统测试指南

## 测试概述
本文档提供了自动驾驶系统各个模块的独立测试和集成测试方法，确保系统的可靠性和性能。

## 测试环境准备

### 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- 至少8GB RAM
- 至少50GB可用磁盘空间
- 支持OpenGL的显卡（用于可视化）

### 环境设置
```bash
# 1. 安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 2. 安装依赖
cd /home/onion/samples/autonomous_driving_ws
chmod +x install_dependencies.sh
./install_dependencies.sh

# 3. 编译系统
colcon build --symlink-install

# 4. 设置环境变量
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 模块独立测试

### 1. auto_msgs 模块测试

#### 基础验证
```bash
# 编译消息包
colcon build --packages-select auto_msgs

# 验证消息定义
ros2 interface show auto_msgs/msg/GridMap
ros2 interface show auto_msgs/msg/PathPoint
ros2 interface show auto_msgs/msg/PlanningPath
ros2 interface show auto_msgs/msg/PlanningRequest
```

#### 功能测试
```bash
# 测试GridMap消息
ros2 topic pub /test_gridmap auto_msgs/msg/GridMap \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, 
    width: 100, height: 100, resolution: 0.1, 
    origin: {position: {x: 0.0, y: 0.0, z: 0.0}}, 
    data: [0, 0, 0]}" --once

# 监听消息
ros2 topic echo /test_gridmap --once

# 测试规划请求消息
ros2 topic pub /test_request auto_msgs/msg/PlanningRequest \
  "{header: {frame_id: 'map'}, 
    start: {pose: {position: {x: 0.0, y: 0.0}}}, 
    goal: {pose: {position: {x: 10.0, y: 10.0}}}, 
    planner_type: 'astar', 
    consider_kinematic: false}" --once
```

#### 验证标准
- ✅ 所有消息类型正确定义
- ✅ 消息可以正常发布和订阅
- ✅ 消息序列化/反序列化正常

### 2. auto_simulation 模块测试

#### 单独测试仿真节点
```bash
# 启动仿真节点
ros2 run auto_simulation simulation_node &

# 等待节点启动
sleep 2

# 检查地图发布
echo "检查地图发布..."
timeout 5 ros2 topic echo /grid_map --once

# 检查规划请求发布
echo "检查规划请求发布..."
timeout 5 ros2 topic echo /planning_request --once

# 重置仿真测试
ros2 topic pub /reset_simulation std_msgs/msg/Empty "{}" --once

# 停止节点
pkill -f simulation_node
```

#### MQTT桥接测试
```bash
# 启动MQTT服务器
sudo systemctl start mosquitto

# 启动MQTT桥接节点
ros2 run auto_simulation mqtt_bridge_node &

# 测试MQTT发布
mosquitto_pub -t "auto_driving/command/request_planning" \
  -m '{"start": {"x": 0, "y": 0}, "goal": {"x": 10, "y": 10}}'

# 测试MQTT订阅
timeout 5 mosquitto_sub -t "auto_driving/planning_path"

# 停止节点
pkill -f mqtt_bridge_node
```

#### 验证标准
- ✅ 地图正确生成和发布
- ✅ 规划请求正确发送
- ✅ MQTT通信正常
- ✅ 可视化正常显示

### 3. auto_perception 模块测试

#### 感知节点测试
```bash
# 启动感知节点
ros2 run auto_perception perception_node &

# 发布测试地图数据
ros2 topic pub /grid_map auto_msgs/msg/GridMap \
  "{header: {frame_id: 'map'}, 
    width: 10, height: 10, resolution: 1.0, 
    data: [0,0,0,0,0,0,0,0,0,0,
           0,100,100,0,0,0,0,0,0,0,
           0,100,100,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0]}" --once

# 检查障碍物检测结果
echo "检查障碍物检测..."
timeout 5 ros2 topic echo /detected_objects --once

# 检查障碍物地图
echo "检查障碍物地图..."
timeout 5 ros2 topic echo /obstacle_map --once

# 停止节点
pkill -f perception_node
```

#### 性能测试
```bash
# 测试处理频率
ros2 run auto_perception perception_node &
sleep 2
echo "测试处理频率..."
timeout 10 ros2 topic hz /detected_objects
pkill -f perception_node
```

#### 验证标准
- ✅ 障碍物正确检测
- ✅ 处理频率 ≥ 5Hz
- ✅ 内存使用 < 500MB
- ✅ CPU使用 < 50%

### 4. auto_planning 模块测试

#### A*算法测试
```bash
# 启动A*规划节点
ros2 run auto_planning path_planner_node --ros-args -p planner_type:=astar &

# 发布测试地图
ros2 topic pub /grid_map auto_msgs/msg/GridMap \
  "{header: {frame_id: 'map'}, 
    width: 20, height: 20, resolution: 0.5, 
    data: $(python3 -c "print([0]*400)")}" --once

# 发送规划请求
ros2 topic pub /planning_request auto_msgs/msg/PlanningRequest \
  "{header: {frame_id: 'map'}, 
    start: {pose: {position: {x: 0.0, y: 0.0}}}, 
    goal: {pose: {position: {x: 9.0, y: 9.0}}}, 
    planner_type: 'astar', 
    consider_kinematic: false}" --once

# 检查规划结果
echo "检查A*规划结果..."
timeout 10 ros2 topic echo /planning_path --once

pkill -f path_planner_node
```

#### Hybrid A*算法测试
```bash
# 启动Hybrid A*规划节点
ros2 run auto_planning path_planner_node --ros-args -p planner_type:=hybrid_astar &

# 发送规划请求
ros2 topic pub /planning_request auto_msgs/msg/PlanningRequest \
  "{header: {frame_id: 'map'}, 
    start: {pose: {position: {x: 0.0, y: 0.0}}}, 
    goal: {pose: {position: {x: 9.0, y: 9.0}}}, 
    planner_type: 'hybrid_astar', 
    consider_kinematic: true}" --once

# 检查规划结果
echo "检查Hybrid A*规划结果..."
timeout 15 ros2 topic echo /planning_path --once

pkill -f path_planner_node
```

#### 单元测试
```bash
# 运行规划模块单元测试
colcon test --packages-select auto_planning
colcon test-result --verbose --test-result-base build/auto_planning
```

#### 验证标准
- ✅ A*规划时间 < 1s
- ✅ Hybrid A*规划时间 < 3s
- ✅ 路径连通性 100%
- ✅ 单元测试全部通过

### 5. auto_control 模块测试

#### 控制节点测试
```bash
# 启动控制节点
ros2 run auto_control controller_node &

# 发布测试路径
ros2 topic pub /planning_path auto_msgs/msg/PlanningPath \
  "{header: {frame_id: 'map'}, 
    points: [
      {pose: {pose: {position: {x: 0.0, y: 0.0}}}, velocity: 1.0},
      {pose: {pose: {position: {x: 1.0, y: 0.0}}}, velocity: 1.0},
      {pose: {pose: {position: {x: 2.0, y: 0.0}}}, velocity: 1.0}
    ], 
    total_length: 2.0, 
    planner_type: 'test'}" --once

# 发布车辆状态
ros2 topic pub /vehicle_state geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, 
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, 
           orientation: {w: 1.0}}}" --once

# 检查控制指令
echo "检查控制指令..."
timeout 5 ros2 topic echo /control_cmd --once

pkill -f controller_node
```

#### 参数调优测试
```bash
# 测试不同前瞻距离
echo "测试前瞻距离参数..."
ros2 run auto_control controller_node --ros-args -p lookahead_distance:=1.5 &
sleep 2
ros2 param get /controller_node lookahead_distance
pkill -f controller_node

# 测试不同最大速度
echo "测试最大速度参数..."
ros2 run auto_control controller_node --ros-args -p max_velocity:=1.5 &
sleep 2
ros2 param get /controller_node max_velocity
pkill -f controller_node
```

#### 单元测试
```bash
# 运行控制模块单元测试
colcon test --packages-select auto_control
colcon test-result --verbose --test-result-base build/auto_control
```

#### 验证标准
- ✅ 控制指令正确生成
- ✅ 控制频率 ≥ 10Hz
- ✅ 参数可以正确设置
- ✅ 单元测试全部通过

## 系统集成测试

### 完整系统启动测试
```bash
# 创建测试脚本
cat > test_full_system.sh << 'EOF'
#!/bin/bash

echo "=== 启动完整系统测试 ==="

# 启动完整系统
ros2 launch auto_simulation auto_driving.launch.xml &
LAUNCH_PID=$!

# 等待系统启动
echo "等待系统启动..."
sleep 10

# 验证所有节点运行
echo "验证节点状态..."
EXPECTED_NODES=("simulation_node" "perception_node" "path_planner_node" "controller_node" "mqtt_bridge_node")
RUNNING_NODES=$(ros2 node list)

for node in "${EXPECTED_NODES[@]}"; do
    if echo "$RUNNING_NODES" | grep -q "$node"; then
        echo "✅ $node 运行正常"
    else
        echo "❌ $node 未运行"
    fi
done

# 检查话题通信
echo "检查话题通信..."
EXPECTED_TOPICS=("/grid_map" "/planning_path" "/control_cmd" "/detected_objects")

for topic in "${EXPECTED_TOPICS[@]}"; do
    if timeout 5 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
        echo "✅ $topic 通信正常"
    else
        echo "❌ $topic 通信异常"
    fi
done

# 检查话题频率
echo "检查话题频率..."
for topic in "${EXPECTED_TOPICS[@]}"; do
    FREQ=$(timeout 10 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$FREQ" ]; then
        echo "✅ $topic 频率: $FREQ Hz"
    else
        echo "❌ $topic 无数据"
    fi
done

# 停止系统
echo "停止系统..."
kill $LAUNCH_PID
wait $LAUNCH_PID 2>/dev/null

echo "=== 系统测试完成 ==="
EOF

chmod +x test_full_system.sh
./test_full_system.sh
```

### 端到端功能测试
```bash
# 运行集成测试
echo "=== 运行端到端测试 ==="
ros2 launch auto_integration_test integration_test.launch.xml &
TEST_PID=$!

# 监控测试进度
echo "监控测试进度..."
timeout 300 ros2 topic echo /test_progress

# 等待测试完成
wait $TEST_PID

# 查看测试结果
if [ -f "integration_test_results.csv" ]; then
    echo "=== 测试结果 ==="
    cat integration_test_results.csv
else
    echo "❌ 测试结果文件未找到"
fi
```

### 性能压力测试
```bash
# 创建性能测试脚本
cat > performance_test.sh << 'EOF'
#!/bin/bash

echo "=== 性能压力测试 ==="

# 启动系统
ros2 launch auto_simulation auto_driving.launch.xml &
LAUNCH_PID=$!
sleep 10

# 监控系统资源
echo "开始资源监控..."
top -b -n1 | grep -E "(simulation_node|perception_node|path_planner_node|controller_node)" > resource_usage.log &
MONITOR_PID=$!

# 发送大量规划请求
echo "发送压力测试请求..."
for i in {1..100}; do
    ros2 topic pub /planning_request auto_msgs/msg/PlanningRequest \
      "{header: {frame_id: 'map'}, 
        start: {pose: {position: {x: $(($i % 10)), y: $(($i % 10))}}}, 
        goal: {pose: {position: {x: $((($i + 5) % 10)), y: $((($i + 5) % 10))}}}, 
        planner_type: 'astar'}" --once
    sleep 0.1
done

# 等待处理完成
sleep 30

# 停止监控
kill $MONITOR_PID 2>/dev/null

# 分析结果
echo "=== 资源使用情况 ==="
cat resource_usage.log

# 停止系统
kill $LAUNCH_PID
wait $LAUNCH_PID 2>/dev/null

echo "=== 性能测试完成 ==="
EOF

chmod +x performance_test.sh
./performance_test.sh
```

## 自动化测试脚本

### 完整测试套件
```bash
# 创建完整测试脚本
cat > run_all_tests.sh << 'EOF'
#!/bin/bash

set -e

echo "========================================="
echo "自动驾驶系统完整测试套件"
echo "========================================="

# 测试结果记录
TEST_RESULTS=()
FAILED_TESTS=()

# 测试函数
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    echo ""
    echo "--- 运行测试: $test_name ---"
    
    if eval "$test_command"; then
        echo "✅ $test_name: 通过"
        TEST_RESULTS+=("✅ $test_name")
    else
        echo "❌ $test_name: 失败"
        TEST_RESULTS+=("❌ $test_name")
        FAILED_TESTS+=("$test_name")
    fi
}

# 1. 编译测试
run_test "系统编译" "colcon build --symlink-install"

# 2. 消息模块测试
run_test "消息模块" "colcon test --packages-select auto_msgs"

# 3. 各模块单元测试
run_test "感知模块单元测试" "colcon test --packages-select auto_perception"
run_test "规划模块单元测试" "colcon test --packages-select auto_planning"
run_test "控制模块单元测试" "colcon test --packages-select auto_control"

# 4. 集成测试
run_test "系统集成测试" "./test_full_system.sh"

# 5. 性能测试
run_test "性能压力测试" "./performance_test.sh"

# 6. 端到端测试
run_test "端到端功能测试" "timeout 300 ros2 launch auto_integration_test integration_test.launch.xml"

# 生成测试报告
echo ""
echo "========================================="
echo "测试结果汇总"
echo "========================================="

for result in "${TEST_RESULTS[@]}"; do
    echo "$result"
done

echo ""
echo "总测试数: ${#TEST_RESULTS[@]}"
echo "失败测试数: ${#FAILED_TESTS[@]}"
echo "成功率: $(( (${#TEST_RESULTS[@]} - ${#FAILED_TESTS[@]}) * 100 / ${#TEST_RESULTS[@]} ))%"

if [ ${#FAILED_TESTS[@]} -eq 0 ]; then
    echo ""
    echo "🎉 所有测试通过！"
    exit 0
else
    echo ""
    echo "❌ 以下测试失败:"
    for failed in "${FAILED_TESTS[@]}"; do
        echo "  - $failed"
    done
    exit 1
fi
EOF

chmod +x run_all_tests.sh
```

### 持续集成测试
```bash
# 创建CI测试脚本
cat > ci_test.sh << 'EOF'
#!/bin/bash

# CI环境测试脚本
set -e

echo "=== CI环境测试 ==="

# 设置环境
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

# 快速编译测试
echo "编译系统..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置环境
source install/setup.bash

# 运行核心测试
echo "运行核心测试..."
colcon test --packages-select auto_msgs auto_planning auto_control

# 检查测试结果
colcon test-result --verbose

echo "=== CI测试完成 ==="
EOF

chmod +x ci_test.sh
```

## 测试数据和场景

### 标准测试场景
```bash
# 创建测试场景目录
mkdir -p test_scenarios

# 场景1: 简单直线路径
cat > test_scenarios/simple_straight.yaml << 'EOF'
scenario_name: "简单直线路径"
description: "无障碍物的直线规划测试"
map:
  width: 20
  height: 20
  resolution: 0.5
  obstacles: []
start_pose:
  x: 0.0
  y: 0.0
  theta: 0.0
goal_pose:
  x: 9.0
  y: 0.0
  theta: 0.0
expected_result:
  success: true
  max_planning_time: 1.0
  path_length_tolerance: 0.1
EOF

# 场景2: 障碍物避让
cat > test_scenarios/obstacle_avoidance.yaml << 'EOF'
scenario_name: "障碍物避让"
description: "静态障碍物绕行测试"
map:
  width: 20
  height: 20
  resolution: 0.5
  obstacles:
    - x: 5
      y: 0
      width: 2
      height: 4
start_pose:
  x: 0.0
  y: 0.0
  theta: 0.0
goal_pose:
  x: 9.0
  y: 0.0
  theta: 0.0
expected_result:
  success: true
  max_planning_time: 3.0
  min_safety_margin: 0.5
EOF
```

## 故障排除指南

### 常见问题诊断
```bash
# 创建诊断脚本
cat > diagnose_system.sh << 'EOF'
#!/bin/bash

echo "=== 系统诊断工具 ==="

# 检查ROS2环境
echo "1. 检查ROS2环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未设置"
else
    echo "✅ ROS2版本: $ROS_DISTRO"
fi

# 检查依赖
echo "2. 检查系统依赖..."
DEPS=("mosquitto" "python3-pip" "cmake")
for dep in "${DEPS[@]}"; do
    if dpkg -l | grep -q "$dep"; then
        echo "✅ $dep 已安装"
    else
        echo "❌ $dep 未安装"
    fi
done

# 检查编译状态
echo "3. 检查编译状态..."
if [ -d "build" ] && [ -d "install" ]; then
    echo "✅ 系统已编译"
else
    echo "❌ 系统未编译，请运行: colcon build"
fi

# 检查节点状态
echo "4. 检查节点状态..."
NODES=$(ros2 node list 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "✅ ROS2通信正常"
    echo "运行中的节点:"
    echo "$NODES"
else
    echo "❌ ROS2通信异常"
fi

# 检查话题状态
echo "5. 检查话题状态..."
TOPICS=$(ros2 topic list 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "✅ 话题通信正常"
    echo "活跃话题数: $(echo "$TOPICS" | wc -l)"
else
    echo "❌ 话题通信异常"
fi

echo "=== 诊断完成 ==="
EOF

chmod +x diagnose_system.sh
```

### 日志分析工具
```bash
# 创建日志分析脚本
cat > analyze_logs.sh << 'EOF'
#!/bin/bash

echo "=== 日志分析工具 ==="

# 分析ROS2日志
if [ -d "$HOME/.ros/log" ]; then
    echo "最近的错误日志:"
    find "$HOME/.ros/log" -name "*.log" -mtime -1 -exec grep -l "ERROR\|FATAL" {} \; | head -5
    
    echo "最近的警告日志:"
    find "$HOME/.ros/log" -name "*.log" -mtime -1 -exec grep -l "WARN" {} \; | head -5
else
    echo "未找到ROS2日志目录"
fi

# 分析系统日志
echo "系统错误日志:"
journalctl --since "1 hour ago" --priority=err --no-pager | tail -10

echo "=== 日志分析完成 ==="
EOF

chmod +x analyze_logs.sh
```

## 测试报告生成

### 自动报告生成
```bash
# 运行完整测试并生成报告
echo "运行完整测试套件..."
./run_all_tests.sh > test_output.log 2>&1

# 生成HTML报告
python3 << 'EOF'
import datetime
import os

# 读取测试输出
with open('test_output.log', 'r') as f:
    test_output = f.read()

# 生成HTML报告
html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>自动驾驶系统测试报告</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; }}
        .header {{ background-color: #f0f0f0; padding: 20px; }}
        .success {{ color: green; }}
        .failure {{ color: red; }}
        .log {{ background-color: #f8f8f8; padding: 10px; font-family: monospace; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>自动驾驶系统测试报告</h1>
        <p>生成时间: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
    </div>
    
    <h2>测试输出</h2>
    <div class="log">
        <pre>{test_output}</pre>
    </div>
    
    <h2>系统信息</h2>
    <ul>
        <li>操作系统: {os.uname().sysname} {os.uname().release}</li>
        <li>Python版本: {os.sys.version}</li>
        <li>工作目录: {os.getcwd()}</li>
    </ul>
</body>
</html>
"""

with open('test_report.html', 'w') as f:
    f.write(html_content)

print("测试报告已生成: test_report.html")
EOF
```

这个完整的测试指南提供了：

1. **环境准备** - 系统要求和环境设置
2. **模块独立测试** - 每个模块的单独测试方法
3. **集成测试** - 完整系统的集成测试
4. **自动化脚本** - 自动化测试执行
5. **故障排除** - 常见问题的诊断和解决
6. **报告生成** - 自动化测试报告生成

通过这个指南，开发者可以系统性地测试整个自动驾驶系统，确保每个模块和整体系统的可靠性。 
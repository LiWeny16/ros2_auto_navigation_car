#!/bin/bash

# 自动驾驶仿真系统测试和优化脚本

echo "开始自动驾驶仿真系统测试和优化..."

# 切换到工作空间目录
cd /home/onion/samples/autonomous_driving_ws

# 编译代码
echo "编译代码..."
colcon build --symlink-install

# 加载环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 运行集成测试
echo "运行集成测试..."
ros2 launch auto_integration_test auto_test.launch.xml test_type:=integration results_file:=integration_test_results.csv

# 运行性能优化
echo "运行性能优化..."
ros2 launch auto_integration_test auto_test.launch.xml test_type:=performance results_file:=performance_optimization_results.csv

echo "测试和优化完成。结果保存在工作空间目录中的CSV文件中。"

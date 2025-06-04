#!/bin/bash

# CARLA 集成测试脚本
# 该脚本用于启动 CARLA 仿真器和 ROS 集成环境

echo "========================================="
echo "CARLA 自动驾驶集成测试启动脚本"
echo "========================================="

# 检查 CARLA 是否已安装
if [ ! -d "/opt/carla-simulator" ] && [ ! -d "$HOME/carla" ]; then
    echo "❌ 未检测到 CARLA 安装"
    echo "请按照以下步骤安装 CARLA:"
    echo "1. 下载 CARLA 0.9.15: https://github.com/carla-simulator/carla/releases"
    echo "2. 解压到 /opt/carla-simulator 或 $HOME/carla"
    echo "3. 重新运行此脚本"
    exit 1
fi

# 设置 CARLA 路径
if [ -d "/opt/carla-simulator" ]; then
    CARLA_PATH="/opt/carla-simulator"
elif [ -d "$HOME/carla" ]; then
    CARLA_PATH="$HOME/carla"
fi

echo "✅ 检测到 CARLA 安装路径: $CARLA_PATH"

# 设置环境变量
export CARLA_ROOT=$CARLA_PATH
export PYTHONPATH=$PYTHONPATH:$CARLA_PATH/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg

# 检查是否已经启动 CARLA 服务器
if pgrep -x "CarlaUE4" > /dev/null; then
    echo "✅ CARLA 服务器已在运行"
else
    echo "🚀 启动 CARLA 服务器..."
    cd $CARLA_PATH
    ./CarlaUE4.sh -quality-level=Low -resx=800 -resy=600 &
    CARLA_PID=$!
    echo "CARLA 服务器 PID: $CARLA_PID"
    
    # 等待 CARLA 服务器启动
    echo "等待 CARLA 服务器启动 (30秒)..."
    sleep 30
fi

# 返回工作空间目录
cd /home/onion/samples/autonomous_driving_ws

echo "🔧 源化 ROS 2 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "🚀 启动 CARLA ROS 集成..."

# 选择启动模式
echo "请选择启动模式:"
echo "1) 完整集成 (CARLA + 所有自动驾驶模块)"
echo "2) 仅 CARLA 集成 (用于测试)"
echo "3) 传感器数据测试"
read -p "请选择 (1-3): " choice

case $choice in
    1)
        echo "启动完整自动驾驶系统..."
        ros2 launch carla_integration carla_integration.launch.py &
        sleep 5
        ros2 launch auto_perception perception.launch.py &
        sleep 3
        ros2 launch auto_planning planning.launch.py &
        sleep 3
        ros2 launch auto_control control.launch.py &
        ;;
    2)
        echo "启动 CARLA 集成测试..."
        ros2 launch carla_integration carla_integration.launch.py
        ;;
    3)
        echo "启动传感器数据测试..."
        ros2 launch carla_integration sensors_only.launch.py
        ;;
    *)
        echo "无效选择，启动默认模式..."
        ros2 launch carla_integration carla_integration.launch.py
        ;;
esac

echo "========================================="
echo "系统已启动！"
echo "========================================="
echo "可用的监控工具:"
echo "- RViz: ros2 run rviz2 rviz2"
echo "- RQT: ros2 run rqt_gui rqt_gui"
echo "- Topic 监控: ros2 topic list"
echo "- 节点监控: ros2 node list"
echo ""
echo "按 Ctrl+C 停止所有节点"

# 等待用户中断
wait

#!/bin/bash

# 自动驾驶仿真系统依赖项安装脚本

echo "开始安装自动驾驶仿真系统所需的依赖项..."

# 更新软件包列表
sudo apt update

# 安装ROS2 Humble依赖项
sudo apt install -y \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-rviz2 \
  ros-humble-visualization-msgs \
  ros-humble-sensor-msgs \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins

# 安装Fast-DDS依赖项
sudo apt install -y \
  ros-humble-fastrtps \
  ros-humble-rmw-fastrtps-cpp

# 安装MQTT依赖项
sudo apt install -y \
  libmosquitto-dev \
  mosquitto \
  mosquitto-clients

# 安装JSON库
sudo apt install -y \
  nlohmann-json3-dev

# 安装编译工具
sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions

echo "所有依赖项安装完成。"
echo "现在可以编译自动驾驶仿真系统："
echo "cd /home/onion/samples/autonomous_driving_ws"
echo "colcon build --symlink-install"
echo "source install/setup.bash"
echo "ros2 launch auto_simulation auto_driving.launch.xml"

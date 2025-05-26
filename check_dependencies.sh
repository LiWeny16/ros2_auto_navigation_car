#!/bin/bash
# 依赖版本管理脚本 - 按照企业规范进行依赖管理

# 定义各个依赖的版本
ROS2_VERSION="humble"
TF2_VERSION="0.25.12"
EIGEN_VERSION="3.4.0"
NLOHMANN_JSON_VERSION="3.10.5"
MOSQUITTO_VERSION="2.0.11"

echo "============================================"
echo "自动驾驶系统依赖版本控制工具"
echo "============================================"
echo "检查系统中安装的依赖版本是否符合兼容性要求..."

# 检查ROS2版本
if [ -f /opt/ros/${ROS2_VERSION}/setup.bash ]; then
  echo "✓ ROS2 ${ROS2_VERSION} 已正确安装"
else
  echo "✗ 错误: 未找到ROS2 ${ROS2_VERSION}，请安装正确版本"
  exit 1
fi

# 检查TF2版本
TF2_INSTALLED_VERSION=$(dpkg -s ros-${ROS2_VERSION}-tf2 | grep "Version" | awk '{print $2}' | cut -d'-' -f1)
if [[ "$TF2_INSTALLED_VERSION" == "$TF2_VERSION" ]]; then
  echo "✓ TF2 版本 ${TF2_VERSION} 已正确安装"
else
  echo "⚠ 警告: TF2版本不匹配，已安装 ${TF2_INSTALLED_VERSION}，建议版本 ${TF2_VERSION}"
fi

# 检查Eigen版本
if pkg-config --exists eigen3; then
  EIGEN_INSTALLED_VERSION=$(pkg-config --modversion eigen3)
  if [[ "$EIGEN_INSTALLED_VERSION" == "$EIGEN_VERSION" ]]; then
    echo "✓ Eigen 版本 ${EIGEN_VERSION} 已正确安装"
  else
    echo "⚠ 警告: Eigen版本不匹配，已安装 ${EIGEN_INSTALLED_VERSION}，建议版本 ${EIGEN_VERSION}"
  fi
else
  echo "✗ 错误: 未找到Eigen，请安装 ${EIGEN_VERSION} 版本"
  exit 1
fi

# 检查nlohmann_json版本
NLOHMANN_JSON_INSTALLED=$(dpkg -s nlohmann-json3-dev | grep "Version" | awk '{print $2}' 2>/dev/null || echo "not_installed")
if [[ "$NLOHMANN_JSON_INSTALLED" != "not_installed" ]]; then
  echo "✓ nlohmann_json 已安装"
else
  echo "✗ 错误: 未找到nlohmann_json库，请安装版本 ${NLOHMANN_JSON_VERSION}"
  exit 1
fi

# 检查libmosquitto版本
MOSQUITTO_INSTALLED_VERSION=$(dpkg -s libmosquitto-dev | grep "Version" | awk '{print $2}' | cut -d'-' -f1 2>/dev/null || echo "not_installed")
if [[ "$MOSQUITTO_INSTALLED_VERSION" != "not_installed" ]]; then
  echo "✓ libmosquitto 版本 ${MOSQUITTO_INSTALLED_VERSION} 已安装"
else
  echo "✗ 错误: 未找到libmosquitto，请安装版本 ${MOSQUITTO_VERSION}"
  exit 1
fi

echo "============================================"
echo "所有必需的依赖已检查完毕"
echo "如有版本不匹配警告，请考虑使用 ./install_dependencies.sh 脚本安装正确版本"
echo "============================================"

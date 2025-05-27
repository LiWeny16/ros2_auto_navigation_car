#!/bin/bash

# 自动驾驶仿真系统依赖项安装脚本
# 版本: 1.0.0
# 遵循语义化版本控制 (SemVer 2.0.0)
# 参考: https://semver.org/

set -euo pipefail  # 严格错误处理

# 定义版本常量
readonly ROS2_VERSION="humble"
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# 日志函数
log_info() {
    echo "[INFO] $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo "[ERROR] $(date '+%Y-%m-%d %H:%M:%S') - $1" >&2
}

log_success() {
    echo "[SUCCESS] $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# 检查是否为root用户
check_root_privileges() {
    if [[ $EUID -eq 0 ]]; then
        log_error "请不要以root用户运行此脚本"
        exit 1
    fi
}

# 安装ROS2依赖
install_ros2_dependencies() {
    log_info "安装ROS2 ${ROS2_VERSION} 依赖项..."
    
    sudo apt update
    
    local ros2_packages=(
        "ros-${ROS2_VERSION}-tf2"
        "ros-${ROS2_VERSION}-tf2-ros"
        "ros-${ROS2_VERSION}-tf2-geometry-msgs"
        "ros-${ROS2_VERSION}-nav-msgs"
        "ros-${ROS2_VERSION}-rviz2"
        "ros-${ROS2_VERSION}-visualization-msgs"
        "ros-${ROS2_VERSION}-sensor-msgs"
        "ros-${ROS2_VERSION}-rqt"
        "ros-${ROS2_VERSION}-rqt-common-plugins"
        "ros-${ROS2_VERSION}-fastrtps"
        "ros-${ROS2_VERSION}-rmw-fastrtps-cpp"
    )
    
    sudo apt install -y "${ros2_packages[@]}"
    log_success "ROS2依赖项安装完成"
}

# 安装MQTT依赖
install_mqtt_dependencies() {
    log_info "安装MQTT依赖项..."
    
    local mqtt_packages=(
        "libmosquitto-dev"
        "mosquitto"
        "mosquitto-clients"
    )
    
    sudo apt install -y "${mqtt_packages[@]}"
    log_success "MQTT依赖项安装完成"
}

# 安装开发工具
install_development_tools() {
    log_info "安装开发工具..."
    
    local dev_packages=(
        "nlohmann-json3-dev"
        "build-essential"
        "cmake"
        "python3-colcon-common-extensions"
    )
    
    sudo apt install -y "${dev_packages[@]}"
    log_success "开发工具安装完成"
}

# 验证安装
verify_installation() {
    log_info "验证安装..."
    
    # 检查ROS2
    if [[ -f "/opt/ros/${ROS2_VERSION}/setup.bash" ]]; then
        log_success "ROS2 ${ROS2_VERSION} 验证通过"
    else
        log_error "ROS2 ${ROS2_VERSION} 验证失败"
        return 1
    fi
    
    # 检查关键包
    local required_commands=("mosquitto_pub" "cmake" "colcon")
    for cmd in "${required_commands[@]}"; do
        if command -v "$cmd" &> /dev/null; then
            log_success "$cmd 验证通过"
        else
            log_error "$cmd 验证失败"
            return 1
        fi
    done
}

# 主函数
main() {
    log_info "开始安装自动驾驶仿真系统所需的依赖项..."
    
    check_root_privileges
    install_ros2_dependencies
    install_mqtt_dependencies
    install_development_tools
    verify_installation
    
    log_success "所有依赖项安装完成"
    echo ""
    echo "下一步操作："
    echo "1. cd ${WORKSPACE_ROOT}"
    echo "2. colcon build --symlink-install"
    echo "3. source install/setup.bash"
    echo "4. ros2 launch auto_simulation auto_driving.launch.xml"
}

# 脚本入口点
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 
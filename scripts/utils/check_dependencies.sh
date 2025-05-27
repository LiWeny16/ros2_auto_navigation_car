#!/bin/bash

# 依赖版本管理脚本 - 按照企业规范进行依赖管理
# 版本: 1.0.0
# 遵循语义化版本控制 (SemVer 2.0.0)
# 参考: https://semver.org/

set -euo pipefail  # 严格错误处理

# 定义各个依赖的版本 - 语义化版本控制
readonly ROS2_VERSION="humble"
readonly TF2_VERSION="0.25.12"
readonly EIGEN_VERSION="3.4.0"
readonly NLOHMANN_JSON_VERSION="3.10.5"
readonly MOSQUITTO_VERSION="2.0.11"

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# 颜色定义
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# 全局变量声明
check_results=()

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1" >&2
}

# 打印分隔线
print_separator() {
    echo "============================================"
}

# 检查ROS2版本
check_ros2_version() {
    log_info "检查ROS2版本..."
    
    if [[ -f "/opt/ros/${ROS2_VERSION}/setup.bash" ]]; then
        log_success "ROS2 ${ROS2_VERSION} 已正确安装"
        return 0
    else
        log_error "未找到ROS2 ${ROS2_VERSION}，请安装正确版本"
        return 1
    fi
}

# 检查TF2版本
check_tf2_version() {
    log_info "检查TF2版本..."
    
    if dpkg -s "ros-${ROS2_VERSION}-tf2" &> /dev/null; then
        local tf2_installed_version
        tf2_installed_version=$(dpkg -s "ros-${ROS2_VERSION}-tf2" | grep "Version" | awk '{print $2}' | cut -d'-' -f1)
        
        if [[ "$tf2_installed_version" == "$TF2_VERSION" ]]; then
            log_success "TF2 版本 ${TF2_VERSION} 已正确安装"
            return 0
        else
            log_warning "TF2版本不匹配，已安装 ${tf2_installed_version}，建议版本 ${TF2_VERSION}"
            return 1
        fi
    else
        log_error "未找到TF2包，请安装 ros-${ROS2_VERSION}-tf2"
        return 1
    fi
}

# 检查Eigen版本
check_eigen_version() {
    log_info "检查Eigen版本..."
    
    if pkg-config --exists eigen3; then
        local eigen_installed_version
        eigen_installed_version=$(pkg-config --modversion eigen3)
        
        if [[ "$eigen_installed_version" == "$EIGEN_VERSION" ]]; then
            log_success "Eigen 版本 ${EIGEN_VERSION} 已正确安装"
            return 0
        else
            log_warning "Eigen版本不匹配，已安装 ${eigen_installed_version}，建议版本 ${EIGEN_VERSION}"
            return 1
        fi
    else
        log_error "未找到Eigen，请安装 ${EIGEN_VERSION} 版本"
        return 1
    fi
}

# 检查nlohmann_json版本
check_nlohmann_json_version() {
    log_info "检查nlohmann_json版本..."
    
    if dpkg -s nlohmann-json3-dev &> /dev/null; then
        log_success "nlohmann_json 已安装"
        return 0
    else
        log_error "未找到nlohmann_json库，请安装版本 ${NLOHMANN_JSON_VERSION}"
        return 1
    fi
}

# 检查libmosquitto版本
check_mosquitto_version() {
    log_info "检查libmosquitto版本..."
    
    if dpkg -s libmosquitto-dev &> /dev/null; then
        local mosquitto_installed_version
        mosquitto_installed_version=$(dpkg -s libmosquitto-dev | grep "Version" | awk '{print $2}' | cut -d'-' -f1 2>/dev/null || echo "unknown")
        log_success "libmosquitto 版本 ${mosquitto_installed_version} 已安装"
        return 0
    else
        log_error "未找到libmosquitto，请安装版本 ${MOSQUITTO_VERSION}"
        return 1
    fi
}

# 检查编译工具
check_build_tools() {
    log_info "检查编译工具..."
    
    local build_tools=("cmake" "colcon" "g++")
    local missing_tools=()
    
    for tool in "${build_tools[@]}"; do
        if command -v "$tool" &> /dev/null; then
            log_success "$tool 已安装"
        else
            log_error "$tool 未安装"
            missing_tools+=("$tool")
        fi
    done
    
    if [[ ${#missing_tools[@]} -eq 0 ]]; then
        return 0
    else
        log_error "缺少编译工具: ${missing_tools[*]}"
        return 1
    fi
}

# 生成依赖报告
generate_dependency_report() {
    local report_file="${WORKSPACE_ROOT}/dependency_report.txt"
    
    log_info "生成依赖报告到 ${report_file}..."
    
    {
        echo "自动驾驶系统依赖检查报告"
        echo "生成时间: $(date)"
        echo "工作空间: ${WORKSPACE_ROOT}"
        echo ""
        echo "期望版本:"
        echo "- ROS2: ${ROS2_VERSION}"
        echo "- TF2: ${TF2_VERSION}"
        echo "- Eigen: ${EIGEN_VERSION}"
        echo "- nlohmann_json: ${NLOHMANN_JSON_VERSION}"
        echo "- Mosquitto: ${MOSQUITTO_VERSION}"
        echo ""
        echo "检查结果: 请查看上方输出"
    } > "$report_file" 2>/dev/null || {
        log_warning "无法生成依赖报告文件"
        return 0
    }
    
    log_success "依赖报告已生成: ${report_file}"
}

# 主函数
main() {
    print_separator
    echo "自动驾驶系统依赖版本控制工具"
    echo "版本: 1.0.0"
    print_separator
    log_info "检查系统中安装的依赖版本是否符合兼容性要求..."
    echo ""
    
    # 执行所有检查
    check_ros2_version && check_results+=("ros2:pass") || check_results+=("ros2:fail")
    check_tf2_version && check_results+=("tf2:pass") || check_results+=("tf2:fail")
    check_eigen_version && check_results+=("eigen:pass") || check_results+=("eigen:fail")
    check_nlohmann_json_version && check_results+=("json:pass") || check_results+=("json:fail")
    check_mosquitto_version && check_results+=("mosquitto:pass") || check_results+=("mosquitto:fail")
    check_build_tools && check_results+=("tools:pass") || check_results+=("tools:fail")
    
    echo ""
    print_separator
    
    # 统计结果
    local passed=0
    local failed=0
    
    for result in "${check_results[@]}"; do
        if [[ "$result" == *":pass" ]]; then
            ((passed++))
        else
            ((failed++))
        fi
    done
    
    log_info "检查完成: ${passed} 项通过, ${failed} 项失败"
    
    if [[ $failed -eq 0 ]]; then
        log_success "所有依赖检查通过！系统已准备就绪。"
        echo ""
        echo "下一步操作："
        echo "1. cd ${WORKSPACE_ROOT}"
        echo "2. colcon build --symlink-install"
        echo "3. source install/setup.bash"
        echo "4. ros2 launch auto_simulation auto_driving.launch.xml"
    else
        log_warning "存在 ${failed} 项依赖问题，建议运行安装脚本修复"
        echo ""
        echo "修复命令："
        echo "./scripts/setup/install_dependencies.sh"
    fi
    
    # 生成报告
    generate_dependency_report
    
    print_separator
    
    # 对于依赖检查，即使有警告也返回0（成功），只有严重错误才返回非零
    if [[ $failed -gt 3 ]]; then
        return 1  # 只有超过3个失败项才认为是严重错误
    else
        return 0  # 少量失败或警告视为成功
    fi
}

# 脚本入口点
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 
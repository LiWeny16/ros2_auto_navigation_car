#!/bin/bash

# 自动驾驶系统启动脚本
# 版本: 2.0.0 - 支持conda环境
# 遵循语义化版本控制 (SemVer 2.0.0)

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# 颜色定义
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1" >&2
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $(date '+%Y-%m-%d %H:%M:%S') - $1"
}

# 检查conda环境
check_conda_env() {
    if [[ -z "${CONDA_DEFAULT_ENV:-}" ]]; then
        log_error "未检测到conda环境，请先激活ros2_auto环境："
        echo "conda activate ros2_auto"
        echo "然后重新运行此脚本"
        exit 1
    fi
    
    if [[ "$CONDA_DEFAULT_ENV" != "ros2_auto" ]]; then
        log_warning "当前conda环境: $CONDA_DEFAULT_ENV，建议使用ros2_auto环境"
    else
        log_success "conda环境检查通过: $CONDA_DEFAULT_ENV"
    fi
}

# 显示帮助信息
show_help() {
    cat << EOF
自动驾驶系统启动脚本 (v2.0.0 - 支持conda环境)

用法: $0 [选项]

选项:
    --planner TYPE      规划器类型 (astar|hybrid_astar|optimized_astar)
    --no-rviz          不启动RViz可视化
    --no-mqtt          不启动MQTT桥接
    --map-size SIZE    地图大小 (默认: 100)
    --help             显示此帮助信息

环境要求:
    - 需要激活ros2_auto conda环境
    - 工作空间已编译

示例:
    conda activate ros2_auto
    $0                                    # 使用默认配置启动
    $0 --planner astar                    # 使用A*规划器
    $0 --no-rviz                         # 不启动可视化
    $0 --planner hybrid_astar --map-size 150  # 自定义配置
EOF
}

# 检查系统状态
check_system_status() {
    log_info "检查系统状态..."
    
    # 检查conda环境
    check_conda_env
    
    # 检查工作空间
    if [[ ! -d "${WORKSPACE_ROOT}/install" ]]; then
        log_error "工作空间未编译，请先运行编译脚本"
        echo "运行: conda activate ros2_auto"
        echo "运行: ./scripts/utils/build_workspace.sh --ordered --clean"
        exit 1
    fi
    
    # 检查ROS2环境
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        log_info "加载ROS2环境..."
        # 设置所有可能的环境变量以避免"未绑定的变量"错误
        export COLCON_TRACE=${COLCON_TRACE:-}
        export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
        export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
        export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-}
        
        # 修复conda环境中libstdc++版本问题，优先使用系统库
        export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
        
        # shellcheck source=/dev/null
        source /opt/ros/humble/setup.bash
        # shellcheck source=/dev/null
        source "${WORKSPACE_ROOT}/install/setup.bash"
    fi
    
    # 检查MQTT服务
    if ! systemctl is-active --quiet mosquitto 2>/dev/null; then
        log_warning "MQTT服务未运行，尝试启动..."
        if command -v systemctl >/dev/null 2>&1; then
            sudo systemctl start mosquitto || {
                log_warning "无法启动MQTT服务，MQTT功能将不可用"
                log_info "安装MQTT: sudo apt install mosquitto mosquitto-clients"
            }
        else
            log_warning "systemctl不可用，请手动启动mosquitto服务"
        fi
    fi
    
    log_success "系统状态检查通过"
}

# 启动系统
launch_system() {
    local planner_type="$1"
    local use_rviz="$2"
    local use_mqtt="$3"
    local map_size="$4"
    
    log_info "启动自动驾驶仿真系统..."
    log_info "配置: 规划器=${planner_type}, RViz=${use_rviz}, MQTT=${use_mqtt}, 地图大小=${map_size}"
    
    cd "${WORKSPACE_ROOT}"
    
    # 确保环境已加载
    # 设置所有可能的环境变量以避免"未绑定的变量"错误
    export COLCON_TRACE=${COLCON_TRACE:-}
    export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
    export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
    export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-}
    
    # 修复conda环境中libstdc++版本问题，优先使用系统库
    export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
    
    source install/setup.bash
    
    # 构建启动命令
    local launch_cmd="ros2 launch auto_simulation auto_driving.launch.xml"
    launch_cmd+=" planner_type:=${planner_type}"
    launch_cmd+=" use_rviz:=${use_rviz}"
    launch_cmd+=" mqtt_enabled:=${use_mqtt}"
    launch_cmd+=" map_size:=${map_size}"
    
    log_info "执行启动命令: ${launch_cmd}"
    
    # 设置信号处理
    trap 'log_info "正在关闭系统..."; pkill -f "ros2 launch" || true; exit 0' SIGINT SIGTERM
    
    # 启动系统
    eval "$launch_cmd"
}

# 主函数
main() {
    local planner_type="hybrid_astar"
    local use_rviz="true"
    local use_mqtt="true"
    local map_size="100"
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --planner)
                planner_type="$2"
                shift 2
                ;;
            --no-rviz)
                use_rviz="false"
                shift
                ;;
            --no-mqtt)
                use_mqtt="false"
                shift
                ;;
            --map-size)
                map_size="$2"
                shift 2
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                echo "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 验证规划器类型
    case $planner_type in
        astar|hybrid_astar|optimized_astar)
            ;;
        *)
            log_error "无效的规划器类型: $planner_type"
            echo "支持的类型: astar, hybrid_astar, optimized_astar"
            exit 1
            ;;
    esac
    
    check_system_status
    launch_system "$planner_type" "$use_rviz" "$use_mqtt" "$map_size"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 
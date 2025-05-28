#!/bin/bash

# 工作空间编译脚本
# 版本: 2.0.0 - 支持conda环境和依赖顺序编译
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

# 包的依赖顺序
readonly PACKAGE_ORDER=(
    "auto_msgs"           # 消息定义包，无依赖，最先编译
    "auto_perception"     # 感知模块，只依赖系统包
    "auto_planning"       # 规划模块，只依赖系统包
    "auto_control"        # 控制模块，依赖auto_msgs
    "auto_simulation"     # 仿真模块，只依赖系统包
    "auto_integration_test"  # 集成测试，依赖所有其他模块
)

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
        exit 1
    fi
    
    if [[ "$CONDA_DEFAULT_ENV" != "ros2_auto" ]]; then
        log_warning "当前conda环境: $CONDA_DEFAULT_ENV，建议使用ros2_auto环境"
        read -p "是否继续？(y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    
    log_success "conda环境检查通过: $CONDA_DEFAULT_ENV"
}

# 安装Python依赖
install_python_deps() {
    log_info "检查Python依赖..."
    
    local required_packages=("empy==3.3.4" "numpy" "lark" "colcon-common-extensions")
    local missing_packages=()
    
    for package in "${required_packages[@]}"; do
        if ! python -c "import ${package%%=*}" 2>/dev/null; then
            missing_packages+=("$package")
        fi
    done
    
    if [[ ${#missing_packages[@]} -gt 0 ]]; then
        log_info "安装缺失的Python包: ${missing_packages[*]}"
        pip install "${missing_packages[@]}"
    else
        log_success "Python依赖检查通过"
    fi
}

# 按依赖顺序编译
build_in_order() {
    local build_type="$1"
    local clean_build="$2"
    local specific_package="$3"  # 新增：指定包参数
    
    # 清理编译结果
    if [[ "$clean_build" == true ]]; then
        if [[ -n "$specific_package" ]]; then
            # 如果指定了包，只清理该包
            log_info "清理包 $specific_package 的编译结果..."
            rm -rf "build/$specific_package" "install/$specific_package" "log/latest_build/$specific_package"
        else
            log_info "清理所有编译结果..."
            rm -rf build/ install/ log/
        fi
    fi
    
    # 验证包是否存在
    log_info "验证包结构..."
    for package in "${PACKAGE_ORDER[@]}"; do
        if [[ ! -d "src/$package" ]]; then
            log_error "包目录不存在: src/$package"
            return 1
        fi
        if [[ ! -f "src/$package/package.xml" ]]; then
            log_error "包配置文件不存在: src/$package/package.xml"
            return 1
        fi
    done
    
    if [[ -n "$specific_package" ]]; then
        log_info "编译包 $specific_package 及其依赖..."
        local colcon_cmd="colcon build --packages-up-to $specific_package --symlink-install"
        colcon_cmd+=" --cmake-args -DCMAKE_BUILD_TYPE=$build_type"
        
        # 特殊包处理
        case "$specific_package" in
            "auto_planning")
                colcon_cmd+=" -DBUILD_TESTING=OFF"
                log_info "auto_planning: 跳过测试编译"
                ;;
            "auto_integration_test")
                if [[ ! -d "install/auto_msgs" ]] || [[ ! -d "install/auto_control" ]]; then
                    log_error "auto_integration_test 缺少必要的前置依赖"
                    return 1
                fi
                ;;
        esac
        
        log_info "执行编译命令: $colcon_cmd"
        if ! eval "$colcon_cmd"; then
            log_error "包 $specific_package 编译失败"
            return 1
        fi
        
        log_success "包 $specific_package 及其依赖编译完成"
    else
        log_info "按依赖顺序编译所有包..."
        log_info "编译顺序: ${PACKAGE_ORDER[*]}"
        
        for package in "${PACKAGE_ORDER[@]}"; do
            log_info "正在编译包: $package"
            
            local colcon_cmd="colcon build --packages-select $package --symlink-install"
            colcon_cmd+=" --cmake-args -DCMAKE_BUILD_TYPE=$build_type"
            
            # 特殊包处理
            case "$package" in
                "auto_planning")
                    colcon_cmd+=" -DBUILD_TESTING=OFF"
                    log_info "auto_planning: 跳过测试编译"
                    ;;
                "auto_integration_test")
                    if [[ ! -d "install/auto_msgs" ]] || [[ ! -d "install/auto_control" ]]; then
                        log_error "auto_integration_test 缺少必要的前置依赖"
                        return 1
                    fi
                    ;;
            esac
            
            log_info "执行编译命令: $colcon_cmd"
            if ! eval "$colcon_cmd"; then
                log_error "包 $package 编译失败"
                return 1
            fi
            
            log_success "包 $package 编译完成"
        done
    fi
    
    # 加载环境
    if [[ -f "install/setup.bash" ]]; then
        set +u
        source install/setup.bash
        set -u
        log_info "已加载环境"
    fi
    
    return 0
}

# 显示帮助信息
show_help() {
    cat << EOF
工作空间编译脚本 (v2.0.0 - 支持conda环境)

用法: $0 [选项]

选项:
    --clean         清理之前的编译结果
    --debug         使用Debug模式编译
    --release       使用Release模式编译 (默认)
    --packages      仅编译指定包 (用逗号分隔)
    --ordered       按依赖顺序编译所有包 (推荐)
    --setup-env     安装conda环境和依赖
    --help          显示此帮助信息

环境要求:
    - 需要激活ros2_auto conda环境
    - ROS2 Humble环境

示例:
    conda activate ros2_auto
    $0 --setup-env                  # 首次使用，安装环境
    $0 --ordered --clean            # 按依赖顺序清理编译
    $0 --packages auto_msgs         # 仅编译指定包
EOF
}

# 设置环境
setup_environment() {
    log_info "设置开发环境..."
    
    # 检查是否已有ros2_auto环境
    if ! conda env list | grep -q "ros2_auto"; then
        log_info "创建ros2_auto conda环境..."
        conda create -n ros2_auto python=3.10 -y
    fi
    
    log_info "请手动执行以下命令："
    echo "conda activate ros2_auto"
    echo "pip install empy==3.3.4 numpy lark colcon-common-extensions"
    echo "然后重新运行此脚本"
}

# 主函数
main() {
    local clean_build=false
    local build_type="Release"
    local specific_packages=""
    local ordered_build=false
    local setup_env=false
    
    # 设置环境变量避免脚本错误
    export COLCON_TRACE=${COLCON_TRACE:-}
    export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
    export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
    export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-}
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --clean)
                clean_build=true
                shift
                ;;
            --debug)
                build_type="Debug"
                shift
                ;;
            --release)
                build_type="Release"
                shift
                ;;
            --packages)
                specific_packages="$2"
                shift 2
                ;;
            --ordered)
                ordered_build=true
                shift
                ;;
            --setup-env)
                setup_env=true
                shift
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
    
    if [[ "$setup_env" == true ]]; then
        setup_environment
        exit 0
    fi
    
    log_info "开始编译工作空间..."
    log_info "编译模式: ${build_type}"
    
    cd "${WORKSPACE_ROOT}"
    
    # 检查conda环境
    check_conda_env
    
    # 安装Python依赖
    install_python_deps
    
    # 检查ROS2环境
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        log_info "加载ROS2环境..."
        source /opt/ros/humble/setup.bash
    fi
    
    # 选择编译方式
    if [[ -n "$specific_packages" ]]; then
        # 编译指定包及其依赖
        IFS=',' read -ra PACKAGES <<< "$specific_packages"
        for package in "${PACKAGES[@]}"; do
            if ! build_in_order "$build_type" "$clean_build" "$package"; then
                log_error "包 $package 编译失败"
                exit 1
            fi
        done
        log_success "指定包编译完成"
    elif [[ "$ordered_build" == true ]]; then
        if build_in_order "$build_type" "$clean_build" ""; then
            log_success "按依赖顺序编译完成"
        else
            log_error "按依赖顺序编译失败"
            exit 1
        fi
    else
        # 标准编译（不推荐）
        log_warning "使用标准编译，建议使用 --ordered 或 --packages 选项"
        local colcon_cmd="colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${build_type}"
        
        if [[ "$clean_build" == true ]]; then
            rm -rf build/ install/ log/
        fi
        
        log_info "执行编译命令: ${colcon_cmd}"
        eval "$colcon_cmd"
    fi
    
    echo ""
    echo "🎉 编译完成！"
    echo ""
    echo "下一步操作："
    echo "source install/setup.bash"
    echo "./scripts/utils/launch_system.sh --planner astar"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 
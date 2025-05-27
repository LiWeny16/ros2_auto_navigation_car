#!/bin/bash

# 自动驾驶仿真系统管理界面
# 版本: 1.0.0
# 遵循语义化版本控制 (SemVer 2.0.0)
# 参考: https://semver.org/
# 
# 注意：AI自动生成，请人工审阅以防止可能的逻辑错误或幻觉现象。
# 来源参考：Clean Architecture - https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html

set -euo pipefail  # 严格错误处理

# 设置环境变量以避免"未绑定的变量"错误
export COLCON_TRACE=${COLCON_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-}

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="${SCRIPT_DIR}"

# 颜色定义
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly PURPLE='\033[0;35m'
readonly NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

# 打印标题
print_title() {
    clear
    echo -e "${CYAN}"
    cat << 'EOF'
╔══════════════════════════════════════════════════════════════════════════════╗
║                        自动驾驶仿真系统管理界面                              ║
║                     Autonomous Driving Simulation Manager                   ║
║                                版本: 1.0.0                                  ║
╚══════════════════════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
}

# 打印分隔线
print_separator() {
    echo -e "${CYAN}════════════════════════════════════════════════════════════════════════════════${NC}"
}

# 显示系统状态
show_system_status() {
    echo -e "${PURPLE}📊 系统状态检查${NC}"
    print_separator
    
    # 检查ROS2环境
    if [[ -n "${ROS_DISTRO:-}" ]]; then
        echo -e "🟢 ROS2环境: ${GREEN}${ROS_DISTRO}${NC}"
    else
        echo -e "🔴 ROS2环境: ${RED}未设置${NC}"
    fi
    
    # 检查工作空间编译状态
    if [[ -d "${WORKSPACE_ROOT}/install" ]]; then
        echo -e "🟢 工作空间: ${GREEN}已编译${NC}"
        
        # 检查各个包的编译状态
        local packages=("auto_msgs" "auto_perception" "auto_planning" "auto_control" "auto_simulation" "auto_integration_test")
        echo -e "   包编译状态:"
        for pkg in "${packages[@]}"; do
            if [[ -d "${WORKSPACE_ROOT}/install/$pkg" ]]; then
                echo -e "   ✅ $pkg: ${GREEN}已编译${NC}"
            else
                echo -e "   ❌ $pkg: ${RED}未编译${NC}"
            fi
        done
    else
        echo -e "🔴 工作空间: ${RED}未编译${NC}"
    fi
    
    # 检查conda环境
    if [[ -n "${CONDA_DEFAULT_ENV:-}" ]]; then
        if [[ "$CONDA_DEFAULT_ENV" == "ros2_auto" ]]; then
            echo -e "🟢 Conda环境: ${GREEN}${CONDA_DEFAULT_ENV}${NC}"
        else
            echo -e "🟡 Conda环境: ${YELLOW}${CONDA_DEFAULT_ENV} (建议使用ros2_auto)${NC}"
        fi
    else
        echo -e "🔴 Conda环境: ${RED}未激活${NC}"
    fi
    
    # 检查MQTT服务
    if systemctl is-active --quiet mosquitto 2>/dev/null; then
        echo -e "🟢 MQTT服务: ${GREEN}运行中${NC}"
    else
        echo -e "🔴 MQTT服务: ${RED}未运行${NC}"
    fi
    
    # 检查ROS节点
    local running_nodes
    running_nodes=$(pgrep -f "ros2" | wc -l)
    if [[ $running_nodes -gt 0 ]]; then
        echo -e "🟢 ROS节点: ${GREEN}${running_nodes} 个节点运行中${NC}"
    else
        echo -e "⚪ ROS节点: ${YELLOW}无节点运行${NC}"
    fi
    
    echo ""
}

# 显示主菜单
show_main_menu() {
    echo -e "${PURPLE}🚀 主菜单${NC}"
    print_separator
    echo -e "${CYAN}环境设置:${NC}"
    echo "  1) 🔧 安装系统依赖"
    echo "  2) 🔍 检查依赖版本"
    echo "  3) 🏗️  编译工作空间"
    echo ""
    echo -e "${CYAN}系统操作:${NC}"
    echo "  4) 🚗 启动仿真系统"
    echo "  5) 🧪 运行测试和优化"
    echo "  6) 📊 查看系统状态"
    echo ""
    echo -e "${CYAN}高级选项:${NC}"
    echo "  7) 📁 脚本目录结构"
    echo "  8) 📖 显示帮助信息"
    echo "  9) 🔄 刷新界面"
    echo "  0) 🚪 退出"
    echo ""
    print_separator
}

# 显示脚本目录结构
show_script_structure() {
    echo -e "${PURPLE}📁 脚本目录结构${NC}"
    print_separator
    
    echo -e "${CYAN}scripts/${NC}"
    echo -e "├── ${YELLOW}setup/${NC}           # 环境设置脚本"
    echo -e "│   └── ${GREEN}install_dependencies.sh${NC}    # 安装系统依赖"
    echo -e "├── ${YELLOW}testing/${NC}         # 测试相关脚本"
    echo -e "│   └── ${GREEN}run_tests_and_optimize.sh${NC}  # 运行测试和优化"
    echo -e "└── ${YELLOW}utils/${NC}           # 实用工具脚本"
    echo -e "    ├── ${GREEN}check_dependencies.sh${NC}      # 检查依赖版本"
    echo -e "    ├── ${GREEN}build_workspace.sh${NC}         # 编译工作空间"
    echo -e "    └── ${GREEN}launch_system.sh${NC}           # 启动仿真系统"
    echo ""
    
    echo -e "${CYAN}使用方法:${NC}"
    echo "• 直接运行: ./scripts/setup/install_dependencies.sh"
    echo "• 通过主界面: 选择对应的菜单选项"
    echo ""
}

# 显示帮助信息
show_help() {
    echo -e "${PURPLE}📖 帮助信息${NC}"
    print_separator
    
    echo -e "${CYAN}系统概述:${NC}"
    echo "这是一个基于ROS2的自动驾驶仿真系统，包含以下模块："
    echo "• auto_msgs        - 消息定义 (最基础，无依赖)"
    echo "• auto_perception  - 环境感知"
    echo "• auto_planning    - 路径规划算法"
    echo "• auto_control     - 车辆控制 (依赖auto_msgs)"
    echo "• auto_simulation  - 仿真环境和MQTT桥接"
    echo "• auto_integration_test - 系统集成测试 (依赖所有模块)"
    echo ""
    
    echo -e "${CYAN}编译顺序:${NC}"
    echo "正确的包编译顺序 (按依赖关系):"
    echo "1. auto_msgs           # 消息定义，无依赖"
    echo "2. auto_perception     # 感知模块"
    echo "3. auto_planning       # 规划模块"
    echo "4. auto_control        # 控制模块，依赖auto_msgs"
    echo "5. auto_simulation     # 仿真模块"
    echo "6. auto_integration_test # 集成测试，依赖所有模块"
    echo ""
    
    echo -e "${CYAN}快速开始:${NC}"
    echo "1. 激活conda环境: conda activate ros2_auto"
    echo "2. 安装依赖: 选择菜单选项 1"
    echo "3. 按依赖顺序编译: 选择菜单选项 3 -> 1"
    echo "4. 启动系统: 选择菜单选项 4"
    echo ""
    
    echo -e "${CYAN}故障排除:${NC}"
    echo "• 依赖问题: 运行依赖检查 (选项 2)"
    echo "• 编译错误: 使用'按依赖顺序清理后编译' (选项 3 -> 2)"
    echo "• 运行问题: 检查系统状态 (选项 6)"
    echo "• 包依赖错误: 确保按正确顺序编译"
    echo ""
    
    echo -e "${CYAN}参考文档:${NC}"
    echo "• 项目文档: docs/目录"
    echo "• AI编程规范: rules.copilot.md"
    echo "• 模块文档: docs/02_auto_simulation_module.md"
    echo ""
}

# 执行脚本
execute_script() {
    local script_path="$1"
    local script_name="$2"
    local allow_warnings="${3:-false}"  # 新增参数，允许警告
    
    if [[ ! -f "$script_path" ]]; then
        log_error "脚本文件不存在: $script_path"
        return 1
    fi
    
    if [[ ! -x "$script_path" ]]; then
        log_info "设置脚本执行权限..."
        chmod +x "$script_path"
    fi
    
    echo ""
    log_info "执行脚本: $script_name"
    print_separator
    
    local exit_code=0
    "$script_path" || exit_code=$?
    
    if [[ $exit_code -eq 0 ]]; then
        log_success "脚本执行完成: $script_name"
    elif [[ "$allow_warnings" == "true" && $exit_code -le 10 ]]; then
        # 对于依赖检查等脚本，允许轻微的警告（退出码1-10）
        log_warning "脚本执行完成但有警告: $script_name (退出码: $exit_code)"
    else
        log_error "脚本执行失败: $script_name (退出码: $exit_code)"
        echo ""
        echo "按任意键继续..."
        read -r
        return 1
    fi
    
    echo ""
    echo "按任意键继续..."
    read -r
}

# 处理用户选择
handle_choice() {
    local choice="$1"
    
    case $choice in
        1)
            execute_script "${WORKSPACE_ROOT}/scripts/setup/install_dependencies.sh" "安装系统依赖"
            ;;
        2)
            execute_script "${WORKSPACE_ROOT}/scripts/utils/check_dependencies.sh" "检查依赖版本" "true"
            ;;
        3)
            echo ""
            log_info "编译选项:"
            echo "1) 按依赖顺序编译所有模块 (推荐)"
            echo "2) 按依赖顺序清理后编译所有模块"
            echo "3) 按依赖顺序Debug模式编译所有模块"
            echo "4) 选择特定模块编译 (单独编译)"
            echo "5) 标准编译 (可能有依赖问题)"
            echo "6) 返回主菜单"
            echo ""
            read -p "请选择编译选项 [1-6]: " build_choice
            
            case $build_choice in
                1)
                    "${WORKSPACE_ROOT}/scripts/utils/build_workspace.sh" --ordered
                    echo "按任意键继续..."
                    read -r
                    ;;
                2)
                    "${WORKSPACE_ROOT}/scripts/utils/build_workspace.sh" --ordered --clean
                    echo "按任意键继续..."
                    read -r
                    ;;
                3)
                    "${WORKSPACE_ROOT}/scripts/utils/build_workspace.sh" --ordered --debug --clean
                    echo "按任意键继续..."
                    read -r
                    ;;
                4)
                    echo ""
                    log_info "请选择要编译的模块:"
                    echo "1) auto_msgs        - 消息定义 (无依赖)"
                    echo "2) auto_perception  - 感知模块"
                    echo "3) auto_planning    - 规划模块"
                    echo "4) auto_control     - 控制模块"
                    echo "5) auto_simulation  - 仿真模块"
                    echo "6) auto_integration_test - 集成测试"
                    echo "7) 组合模块 (多选)"
                    echo "8) 返回编译菜单"
                    echo ""
                    read -p "请选择模块 [1-8]: " module_choice
                    
                    local package=""
                    local clean_option=""
                    local debug_option=""
                    
                    # 询问是否需要清理
                    echo ""
                    read -p "是否需要清理后编译? [y/N]: " clean_answer
                    if [[ "$clean_answer" == "y" || "$clean_answer" == "Y" ]]; then
                        clean_option="--clean"
                        log_info "将在编译前清理模块"
                    fi
                    
                    # 询问是否需要Debug模式
                    echo ""
                    read -p "是否需要Debug模式编译? [y/N]: " debug_answer
                    if [[ "$debug_answer" == "y" || "$debug_answer" == "Y" ]]; then
                        debug_option="--debug"
                        log_info "将使用Debug模式编译"
                    fi
                    
                    case $module_choice in
                        1)
                            package="auto_msgs"
                            log_info "编译模块: $package"
                            build_dependency_chain "$package" "$clean_option" "$debug_option"
                            ;;
                        2)
                            package="auto_perception"
                            log_info "编译模块: $package"
                            build_dependency_chain "$package" "$clean_option" "$debug_option"
                            ;;
                        3)
                            package="auto_planning"
                            log_info "编译模块: $package"
                            build_dependency_chain "$package" "$clean_option" "$debug_option"
                            ;;
                        4)
                            package="auto_control"
                            log_info "编译模块: $package"
                            colcon build --packages-select $package $debug_option $clean_option
                            ;;
                        5)
                            package="auto_simulation"
                            log_info "编译模块: $package"
                            colcon build --packages-select $package $debug_option $clean_option
                            ;;
                        6)
                            package="auto_integration_test"
                            log_info "编译模块: $package"
                            colcon build --packages-select $package $debug_option $clean_option
                            ;;
                        7)
                            echo ""
                            log_info "请选择要编译的多个模块 (空格分隔, 例如: 1 3 4)"
                            echo "1) auto_msgs"
                            echo "2) auto_perception"
                            echo "3) auto_planning"
                            echo "4) auto_control"
                            echo "5) auto_simulation"
                            echo "6) auto_integration_test"
                            echo ""
                            read -p "请输入模块编号: " -a module_numbers
                            
                            local packages=()
                            local all_modules=("auto_msgs" "auto_perception" "auto_planning" "auto_control" "auto_simulation" "auto_integration_test")
                            
                            for num in "${module_numbers[@]}"; do
                                if [[ "$num" -ge 1 && "$num" -le 6 ]]; then
                                    packages+=("${all_modules[$((num-1))]}")
                                fi
                            done
                            
                            if [[ ${#packages[@]} -gt 0 ]]; then
                                log_info "编译模块: ${packages[*]}"
                                colcon build --packages-select "${packages[@]}" $debug_option $clean_option
                            else
                                log_error "未选择任何模块"
                            fi
                            ;;
                        8)
                            # 返回编译菜单
                            continue
                            ;;
                        *)
                            log_error "无效选择"
                            ;;
                    esac
                    
                    echo "按任意键继续..."
                    read -r
                    ;;
                5)
                    log_warning "标准编译可能会遇到依赖问题，建议使用选项1"
                    execute_script "${WORKSPACE_ROOT}/scripts/utils/build_workspace.sh" "标准编译"
                    ;;
                6)
                    return
                    ;;
                *)
                    log_error "无效选择"
                    ;;
            esac
            ;;
        4)
            echo ""
            log_info "启动选项:"
            echo "1) 默认配置启动"
            echo "2) 使用A*规划器"
            echo "3) 不启动RViz"
            echo "4) 自定义配置"
            echo "5) 返回主菜单"
            echo ""
            read -p "请选择启动选项 [1-5]: " launch_choice
            
            case $launch_choice in
                1)
                    "${WORKSPACE_ROOT}/scripts/utils/launch_system.sh"
                    ;;
                2)
                    "${WORKSPACE_ROOT}/scripts/utils/launch_system.sh" --planner astar
                    ;;
                3)
                    "${WORKSPACE_ROOT}/scripts/utils/launch_system.sh" --no-rviz
                    ;;
                4)
                    echo ""
                    read -p "规划器类型 [astar/hybrid_astar/optimized_astar]: " planner
                    read -p "地图大小 [默认100]: " mapsize
                    mapsize=${mapsize:-100}
                    
                    "${WORKSPACE_ROOT}/scripts/utils/launch_system.sh" --planner "$planner" --map-size "$mapsize"
                    ;;
                5)
                    return
                    ;;
                *)
                    log_error "无效选择"
                    ;;
            esac
            ;;
        5)
            execute_script "${WORKSPACE_ROOT}/scripts/testing/run_tests_and_optimize.sh" "运行测试和优化"
            ;;
        6)
            show_system_status
            echo "按任意键继续..."
            read -r
            ;;
        7)
            show_script_structure
            echo "按任意键继续..."
            read -r
            ;;
        8)
            show_help
            echo "按任意键继续..."
            read -r
            ;;
        9)
            # 刷新界面，什么都不做
            ;;
        0)
            log_info "感谢使用自动驾驶仿真系统！"
            exit 0
            ;;
        *)
            log_error "无效选择，请输入 0-9"
            sleep 1
            ;;
    esac
}

# 单独编译特定目标函数
build_specific_target() {
    local package="$1"
    local target="$2"
    local clean_option="$3"
    local debug_option="$4"
    
    if [[ -z "$package" ]]; then
        log_error "未指定包名"
        return 1
    fi
    
    if [[ -z "$target" ]]; then
        log_info "编译整个包: $package"
        colcon build --packages-select "$package" $debug_option $clean_option
    else
        log_info "编译特定目标: $package::$target"
        colcon build --packages-select "$package" --cmake-target "$target" $debug_option $clean_option
    fi
}

# 编译依赖链函数
build_dependency_chain() {
    local package="$1"
    local clean_option="$2"
    local debug_option="$3"
    
    log_info "按依赖链编译: $package 及其依赖"
    
    # 根据依赖关系确定编译顺序
    local deps=()
    
    case "$package" in
        "auto_msgs")
            deps=("auto_msgs")
            ;;
        "auto_perception")
            deps=("auto_msgs" "auto_perception")
            ;;
        "auto_planning")
            deps=("auto_msgs" "auto_perception" "auto_planning")
            ;;
        "auto_control")
            deps=("auto_msgs" "auto_control")
            ;;
        "auto_simulation")
            deps=("auto_msgs" "auto_perception" "auto_planning" "auto_control" "auto_simulation")
            ;;
        "auto_integration_test")
            deps=("auto_msgs" "auto_perception" "auto_planning" "auto_control" "auto_simulation" "auto_integration_test")
            ;;
        *)
            log_error "未知的包: $package"
            return 1
            ;;
    esac
    
    log_info "将按以下顺序编译: ${deps[*]}"
    colcon build --packages-select "${deps[@]}" $debug_option $clean_option
}

# 主循环
main() {
    # 设置脚本执行权限
    find "${WORKSPACE_ROOT}/scripts" -name "*.sh" -exec chmod +x {} \; 2>/dev/null || true
    
    while true; do
        print_title
        show_main_menu
        read -p "请选择操作 [0-9]: " choice
        handle_choice "$choice"
    done
}

# 脚本入口点
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 
#!/bin/bash

# MQTT服务器启动脚本
# 用于启动本地Eclipse Mosquitto MQTT服务器
# 支持自动安装、配置和启动

set -e  # 遇到错误立即退出

# 脚本信息
SCRIPT_NAME="MQTT服务器启动脚本"
VERSION="1.0.0"
AUTHOR="自动驾驶仿真系统"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# MQTT配置
MQTT_PORT=1883
MQTT_WS_PORT=9001
MQTT_CONFIG_DIR="/etc/mosquitto"
MQTT_CONFIG_FILE="$MQTT_CONFIG_DIR/mosquitto.conf"
MQTT_LOG_DIR="/var/log/mosquitto"
MQTT_PID_FILE="/var/run/mosquitto.pid"

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_debug() {
    echo -e "${BLUE}[DEBUG]${NC} $1"
}

log_header() {
    echo -e "${PURPLE}=================================${NC}"
    echo -e "${PURPLE} $1 ${NC}"
    echo -e "${PURPLE}=================================${NC}"
}

# 显示帮助信息
show_help() {
    cat << EOF
${SCRIPT_NAME} v${VERSION}

用法: $0 [选项]

选项:
    -h, --help          显示此帮助信息
    -i, --install       安装Eclipse Mosquitto MQTT服务器
    -s, --start         启动MQTT服务器
    -t, --stop          停止MQTT服务器
    -r, --restart       重启MQTT服务器
    -c, --status        查看MQTT服务器状态
    -l, --logs          查看MQTT服务器日志
    -p, --port PORT     设置MQTT端口 (默认: 1883)
    -w, --ws-port PORT  设置WebSocket端口 (默认: 9001)
    --config            显示配置信息
    --test              测试MQTT连接
    --clean             清理配置和日志文件
    --daemon            以守护进程方式启动

示例:
    $0 --install        # 安装Mosquitto
    $0 --start          # 启动服务器
    $0 --status         # 查看状态
    $0 --test           # 测试连接
    $0 --logs           # 查看日志

自动驾驶系统MQTT主题:
    - auto_driving/planning_path        # 规划路径
    - auto_driving/vehicle_state        # 车辆状态
    - auto_driving/map_data            # 地图数据
    - auto_driving/command/request_planning  # 规划请求
    - auto_driving/command/manual_control    # 手动控制
    - auto_driving/sensor/obstacles     # 障碍物信息

EOF
}

# 检查是否为root用户
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_warn "检测到root用户，某些操作可能需要额外权限"
        return 0
    else
        return 1
    fi
}

# 检查系统类型
detect_os() {
    if command -v apt-get &> /dev/null; then
        echo "ubuntu"
    elif command -v yum &> /dev/null; then
        echo "centos"
    elif command -v pacman &> /dev/null; then
        echo "arch"
    elif command -v brew &> /dev/null; then
        echo "macos"
    else
        echo "unknown"
    fi
}

# 安装Mosquitto
install_mosquitto() {
    log_header "安装Eclipse Mosquitto MQTT服务器"
    
    local os_type=$(detect_os)
    
    case $os_type in
        "ubuntu")
            log_info "检测到Ubuntu系统，使用apt安装"
            sudo apt update
            sudo apt install -y mosquitto mosquitto-clients
            ;;
        "centos")
            log_info "检测到CentOS系统，使用yum安装"
            sudo yum install -y epel-release
            sudo yum install -y mosquitto mosquitto-clients
            ;;
        "arch")
            log_info "检测到Arch系统，使用pacman安装"
            sudo pacman -S --noconfirm mosquitto
            ;;
        "macos")
            log_info "检测到macOS系统，使用brew安装"
            if ! command -v brew &> /dev/null; then
                log_error "请先安装Homebrew: https://brew.sh/"
                exit 1
            fi
            brew install mosquitto
            MQTT_CONFIG_DIR="/opt/homebrew/etc/mosquitto"
            MQTT_CONFIG_FILE="$MQTT_CONFIG_DIR/mosquitto.conf"
            MQTT_LOG_DIR="/opt/homebrew/var/log/mosquitto"
            ;;
        *)
            log_error "不支持的操作系统，请手动安装Mosquitto"
            log_info "请访问: https://mosquitto.org/download/"
            exit 1
            ;;
    esac
    
    # 验证安装
    if command -v mosquitto &> /dev/null; then
        local version=$(mosquitto -h | head -n 1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+')
        log_info "Mosquitto安装成功，版本: $version"
    else
        log_error "Mosquitto安装失败"
        exit 1
    fi
    
    # 创建配置
    create_config
}

# 创建Mosquitto配置文件
create_config() {
    log_info "创建MQTT服务器配置文件"
    
    # 确保配置目录存在
    if [[ ! -d "$MQTT_CONFIG_DIR" ]]; then
        sudo mkdir -p "$MQTT_CONFIG_DIR"
    fi
    
    # 确保日志目录存在
    if [[ ! -d "$MQTT_LOG_DIR" ]]; then
        sudo mkdir -p "$MQTT_LOG_DIR"
    fi
    
    # 创建配置文件
    sudo tee "$MQTT_CONFIG_FILE" > /dev/null << EOF
# Eclipse Mosquitto MQTT服务器配置
# 自动驾驶仿真系统专用配置

# 基本设置
pid_file $MQTT_PID_FILE

# 网络设置
port $MQTT_PORT
bind_address 0.0.0.0

# WebSocket支持 (用于Web前端)
listener $MQTT_WS_PORT
protocol websockets

# 持久化设置
persistence true
persistence_location /var/lib/mosquitto/

# 日志设置
log_dest file $MQTT_LOG_DIR/mosquitto.log
log_type error
log_type warning
log_type notice
log_type information
log_timestamp true

# 连接设置
max_connections 1000
max_inflight_messages 100
max_queued_messages 1000

# 允许匿名连接 (开发环境)
allow_anonymous true

# 消息大小限制
message_size_limit 1048576

# Keep alive设置
keepalive_interval 60

# 重试设置
retry_interval 20
sys_interval 10

# 自动驾驶系统特定设置
# 主题权限控制 (生产环境可启用)
# acl_file /etc/mosquitto/aclfile

EOF
    
    log_info "配置文件已创建: $MQTT_CONFIG_FILE"
    log_info "MQTT端口: $MQTT_PORT"
    log_info "WebSocket端口: $MQTT_WS_PORT"
}

# 启动MQTT服务器
start_mqtt() {
    log_header "启动MQTT服务器"
    
    # 检查是否已在运行
    if pgrep -x "mosquitto" > /dev/null; then
        log_warn "MQTT服务器已在运行"
        show_status
        return 0
    fi
    
    # 检查配置文件
    if [[ ! -f "$MQTT_CONFIG_FILE" ]]; then
        log_warn "配置文件不存在，创建默认配置"
        create_config
    fi
    
    # 启动服务器
    if [[ "$1" == "--daemon" ]]; then
        log_info "以守护进程方式启动MQTT服务器"
        mosquitto -c "$MQTT_CONFIG_FILE" -d
    else
        log_info "启动MQTT服务器 (前台运行)"
        log_info "按Ctrl+C停止服务器"
        mosquitto -c "$MQTT_CONFIG_FILE"
    fi
}

# 停止MQTT服务器
stop_mqtt() {
    log_header "停止MQTT服务器"
    
    if pgrep -x "mosquitto" > /dev/null; then
        log_info "正在停止MQTT服务器..."
        pkill -x mosquitto
        sleep 2
        
        if ! pgrep -x "mosquitto" > /dev/null; then
            log_info "MQTT服务器已停止"
        else
            log_warn "强制终止MQTT服务器"
            pkill -9 -x mosquitto
        fi
    else
        log_info "MQTT服务器未运行"
    fi
}

# 重启MQTT服务器
restart_mqtt() {
    log_header "重启MQTT服务器"
    stop_mqtt
    sleep 1
    start_mqtt --daemon
}

# 显示MQTT服务器状态
show_status() {
    log_header "MQTT服务器状态"
    
    if pgrep -x "mosquitto" > /dev/null; then
        local pid=$(pgrep -x "mosquitto")
        local uptime=$(ps -o etime= -p $pid | tr -d ' ')
        echo -e "${GREEN}状态:${NC} 运行中"
        echo -e "${GREEN}PID:${NC} $pid"
        echo -e "${GREEN}运行时间:${NC} $uptime"
        echo -e "${GREEN}端口:${NC} $MQTT_PORT (MQTT), $MQTT_WS_PORT (WebSocket)"
        
        # 检查端口监听
        if command -v netstat &> /dev/null; then
            local mqtt_listen=$(netstat -ln | grep ":$MQTT_PORT " | wc -l)
            local ws_listen=$(netstat -ln | grep ":$MQTT_WS_PORT " | wc -l)
            echo -e "${GREEN}MQTT端口监听:${NC} $([[ $mqtt_listen -gt 0 ]] && echo '是' || echo '否')"
            echo -e "${GREEN}WebSocket端口监听:${NC} $([[ $ws_listen -gt 0 ]] && echo '是' || echo '否')"
        fi
    else
        echo -e "${RED}状态:${NC} 未运行"
    fi
    
    # 配置信息
    echo -e "${BLUE}配置文件:${NC} $MQTT_CONFIG_FILE"
    echo -e "${BLUE}日志目录:${NC} $MQTT_LOG_DIR"
}

# 查看日志
show_logs() {
    log_header "MQTT服务器日志"
    
    local log_file="$MQTT_LOG_DIR/mosquitto.log"
    
    if [[ -f "$log_file" ]]; then
        echo -e "${BLUE}日志文件:${NC} $log_file"
        echo -e "${BLUE}最近20行日志:${NC}"
        echo "----------------------------------------"
        tail -n 20 "$log_file"
    else
        log_warn "日志文件不存在: $log_file"
    fi
}

# 测试MQTT连接
test_mqtt() {
    log_header "测试MQTT连接"
    
    if ! command -v mosquitto_pub &> /dev/null || ! command -v mosquitto_sub &> /dev/null; then
        log_error "mosquitto客户端工具未安装"
        log_info "请运行: $0 --install"
        exit 1
    fi
    
    if ! pgrep -x "mosquitto" > /dev/null; then
        log_error "MQTT服务器未运行"
        log_info "请运行: $0 --start"
        exit 1
    fi
    
    local test_topic="auto_driving/test"
    local test_message="Hello from autonomous driving system - $(date)"
    
    log_info "发布测试消息到主题: $test_topic"
    log_info "消息内容: $test_message"
    
    # 在后台启动订阅者
    timeout 5 mosquitto_sub -h localhost -p $MQTT_PORT -t "$test_topic" &
    local sub_pid=$!
    
    sleep 1
    
    # 发布消息
    mosquitto_pub -h localhost -p $MQTT_PORT -t "$test_topic" -m "$test_message"
    
    sleep 2
    kill $sub_pid 2>/dev/null || true
    
    log_info "测试完成，请检查上方是否收到消息"
    
    # 测试WebSocket
    log_info "WebSocket端口 $MQTT_WS_PORT 已配置，可用于Web前端连接"
}

# 显示配置信息
show_config() {
    log_header "MQTT服务器配置信息"
    
    echo -e "${BLUE}服务器配置:${NC}"
    echo "  MQTT端口: $MQTT_PORT"
    echo "  WebSocket端口: $MQTT_WS_PORT"
    echo "  配置文件: $MQTT_CONFIG_FILE"
    echo "  日志目录: $MQTT_LOG_DIR"
    echo "  PID文件: $MQTT_PID_FILE"
    
    echo -e "\n${BLUE}自动驾驶系统MQTT主题:${NC}"
    echo "  auto_driving/planning_path         # 规划路径数据"
    echo "  auto_driving/vehicle_state         # 车辆状态信息"
    echo "  auto_driving/map_data             # 地图数据"
    echo "  auto_driving/command/request_planning   # 路径规划请求"
    echo "  auto_driving/command/manual_control     # 手动控制指令"
    echo "  auto_driving/sensor/obstacles      # 障碍物检测数据"
    
    echo -e "\n${BLUE}连接示例:${NC}"
    echo "  发布消息: mosquitto_pub -h localhost -p $MQTT_PORT -t 'auto_driving/test' -m 'hello'"
    echo "  订阅消息: mosquitto_sub -h localhost -p $MQTT_PORT -t 'auto_driving/#'"
    echo "  Web连接: ws://localhost:$MQTT_WS_PORT"
}

# 清理配置和日志
clean_mqtt() {
    log_header "清理MQTT配置和日志"
    
    read -p "确定要清理所有MQTT配置和日志文件吗? [y/N]: " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        stop_mqtt
        
        log_info "删除配置文件..."
        sudo rm -f "$MQTT_CONFIG_FILE"
        
        log_info "删除日志文件..."
        sudo rm -rf "$MQTT_LOG_DIR"
        
        log_info "删除PID文件..."
        sudo rm -f "$MQTT_PID_FILE"
        
        log_info "清理完成"
    else
        log_info "取消清理操作"
    fi
}

# 主函数
main() {
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -i|--install)
                install_mosquitto
                exit 0
                ;;
            -s|--start)
                start_mqtt
                exit 0
                ;;
            -t|--stop)
                stop_mqtt
                exit 0
                ;;
            -r|--restart)
                restart_mqtt
                exit 0
                ;;
            -c|--status)
                show_status
                exit 0
                ;;
            -l|--logs)
                show_logs
                exit 0
                ;;
            -p|--port)
                MQTT_PORT="$2"
                shift 2
                ;;
            -w|--ws-port)
                MQTT_WS_PORT="$2"
                shift 2
                ;;
            --config)
                show_config
                exit 0
                ;;
            --test)
                test_mqtt
                exit 0
                ;;
            --clean)
                clean_mqtt
                exit 0
                ;;
            --daemon)
                start_mqtt --daemon
                exit 0
                ;;
            *)
                log_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 如果没有参数，显示帮助
    show_help
}

# 运行主函数
main "$@"

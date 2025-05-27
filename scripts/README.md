# Scripts 目录说明

本目录包含自动驾驶仿真系统的所有管理脚本，支持conda环境和按依赖顺序编译。

## 🚀 快速开始

**推荐方式：使用快速设置脚本**
```bash
# 完整设置（推荐新用户）
./scripts/quick_setup.sh --full

# 仅设置conda环境
./scripts/quick_setup.sh --env-only

# 仅编译项目
./scripts/quick_setup.sh --build-only
```

## 目录结构

```
scripts/
├── quick_setup.sh              # 🆕 快速设置脚本（推荐）
├── setup/                      # 环境设置脚本
│   └── install_dependencies.sh # 安装系统依赖
├── testing/                    # 测试相关脚本
│   └── run_tests_and_optimize.sh # 运行测试和优化
└── utils/                      # 实用工具脚本
    ├── check_dependencies.sh   # 检查依赖版本
    ├── build_workspace.sh      # 🔄 编译工作空间（支持conda+顺序编译）
    └── launch_system.sh        # 🔄 启动仿真系统（支持conda）
```

## 🆕 新特性 (v2.0.0)

### Conda环境支持
- 🐍 **独立Python环境**: 使用`ros2_auto` conda环境，避免系统Python冲突
- 📦 **依赖管理**: 自动安装和管理ROS2所需的Python包
- 🔒 **版本控制**: 固定empy版本(3.3.4)确保编译兼容性

### 按依赖顺序编译
- 📋 **预定义顺序**: auto_msgs → auto_perception → auto_planning → auto_control → auto_simulation
- 🔄 **增量编译**: 每个包编译完成后立即加载环境
- 🚫 **跳过测试**: 自动跳过有问题的测试，专注核心功能

## 脚本说明

### 🚀 快速设置脚本

#### quick_setup.sh (🆕)
- **功能**: 一键设置完整开发环境
- **用法**: 
  ```bash
  ./scripts/quick_setup.sh --full        # 完整设置
  ./scripts/quick_setup.sh --env-only    # 仅环境设置
  ./scripts/quick_setup.sh --build-only  # 仅编译
  ```
- **特点**: 
  - 自动检查系统要求
  - 创建和配置conda环境
  - 按依赖顺序编译
  - 可选择启动系统

### 实用工具 (utils/)

#### build_workspace.sh (🔄 已更新)
- **功能**: 支持conda环境的智能编译系统
- **新特性**:
  - 🐍 Conda环境检查和依赖安装
  - 📋 按依赖顺序编译 (`--ordered`)
  - 🧹 清理编译 (`--clean`)
  - 🚫 跳过问题测试
- **用法**: 
  ```bash
  # 推荐：按依赖顺序编译
  conda activate ros2_auto
  ./scripts/utils/build_workspace.sh --ordered --clean
  
  # 编译指定包
  ./scripts/utils/build_workspace.sh --packages auto_msgs,auto_perception
  
  # Debug模式
  ./scripts/utils/build_workspace.sh --debug --ordered
  ```

#### launch_system.sh (🔄 已更新)
- **功能**: 支持conda环境的系统启动
- **新特性**:
  - 🐍 Conda环境验证
  - 🔧 改进的MQTT服务处理
  - 📋 详细的环境检查
- **用法**:
  ```bash
  conda activate ros2_auto
  ./scripts/utils/launch_system.sh --planner astar      # A*规划器
  ./scripts/utils/launch_system.sh --no-rviz           # 无可视化
  ./scripts/utils/launch_system.sh --no-mqtt           # 无MQTT
  ```

#### check_dependencies.sh
- **功能**: 检查系统依赖版本兼容性
- **用法**: `./scripts/utils/check_dependencies.sh`
- **输出**: 依赖检查报告

### 环境设置 (setup/)

#### install_dependencies.sh
- **功能**: 自动安装系统所需的所有依赖项
- **用法**: `./scripts/setup/install_dependencies.sh`
- **包含**: ROS2包、MQTT库、开发工具等

### 测试工具 (testing/)

#### run_tests_and_optimize.sh
- **功能**: 运行完整的测试套件和性能优化
- **用法**: `./scripts/testing/run_tests_and_optimize.sh`
- **包含**: 单元测试、集成测试、性能测试

## 🛠️ 环境要求

### 系统要求
- Ubuntu 20.04 / 22.04
- ROS2 Humble
- Conda/Miniconda

### Python环境
- Python 3.10 (conda环境)
- empy==3.3.4 (兼容ROS2 Humble)
- numpy, lark, colcon-common-extensions

## 使用方法

### 🥇 方法1: 快速设置 (推荐新用户)
```bash
# 一键完整设置
./scripts/quick_setup.sh --full
```

### 🥈 方法2: 分步设置 (推荐开发者)
```bash
# 1. 设置conda环境
./scripts/quick_setup.sh --env-only

# 2. 激活环境
conda activate ros2_auto

# 3. 按依赖顺序编译
./scripts/utils/build_workspace.sh --ordered --clean

# 4. 启动系统
./scripts/utils/launch_system.sh --planner astar
```

### 🥉 方法3: 传统方式
```bash
# 创建conda环境
conda create -n ros2_auto python=3.10 -y
conda activate ros2_auto
pip install empy==3.3.4 numpy lark colcon-common-extensions

# 编译和启动
source /opt/ros/humble/setup.bash
./scripts/utils/build_workspace.sh --ordered
./scripts/utils/launch_system.sh
```

## 📋 编译依赖顺序

系统包按以下顺序编译，确保依赖关系正确：

1. **auto_msgs** - 消息定义（基础依赖）
2. **auto_perception** - 感知模块
3. **auto_planning** - 规划模块
4. **auto_control** - 控制模块  
5. **auto_simulation** - 仿真模块（集成所有）

## 设计原则

本脚本系统遵循以下设计原则：

1. **环境隔离**: 使用conda环境避免系统污染
2. **依赖管理**: 按顺序编译，确保依赖关系
3. **容错性**: 自动跳过问题测试，专注核心功能
4. **模块化**: 按功能分类，职责明确
5. **可重用**: 每个脚本可独立运行
6. **错误处理**: 使用 `set -euo pipefail` 严格错误处理
7. **日志记录**: 统一的日志格式和颜色编码
8. **版本控制**: 遵循语义化版本控制 (SemVer)

## 🔧 故障排除

### Conda环境问题
```bash
# 重新创建环境
conda remove -n ros2_auto --all
./scripts/quick_setup.sh --env-only
```

### 编译问题
```bash
# 检查conda环境
conda activate ros2_auto
conda list | grep empy  # 应该显示 empy==3.3.4

# 清理重新按顺序编译
./scripts/utils/build_workspace.sh --ordered --clean
```

### Python包版本冲突
```bash
# 重新安装正确版本
conda activate ros2_auto
pip uninstall empy -y
pip install empy==3.3.4
```

### 权限问题
```bash
# 设置执行权限
chmod +x scripts/**/*.sh
```

### RViz显示问题
```bash
# 检查环境加载
conda activate ros2_auto
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep auto  # 应该显示所有auto包
```

## 📚 参考文档

- [AI编程规范](../rules.copilot.md)
- [系统文档](../docs/)
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [Conda用户指南](https://docs.conda.io/projects/conda/en/latest/user-guide/)
- [Clean Architecture](https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html) 
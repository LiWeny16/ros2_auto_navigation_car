#!/bin/bash

# 该脚本在sourcing setup.bash时自动执行
# 用于设置规划模块的环境变量

# 设置规划相关变量
export AUTO_PLANNING_CONFIG_PATH=$AMENT_CURRENT_PREFIX/share/auto_planning/config
export AUTO_PLANNING_MAPS_PATH=$AMENT_CURRENT_PREFIX/share/auto_planning/maps

# 设置地图相关变量
export AUTO_PLANNING_DEFAULT_MAP=default_map

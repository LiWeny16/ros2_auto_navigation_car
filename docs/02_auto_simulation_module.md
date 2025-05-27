# auto_simulation 模块文档

## 模块概述
auto_simulation 是仿真环境模块，负责生成模拟环境、发送规划请求、提供MQTT通信桥接功能。

## 模块结构
```
auto_simulation/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── simulation_node.cpp      # 主仿真节点
│   ├── mqtt_bridge.cpp          # MQTT桥接实现
│   └── mqtt_bridge_node.cpp     # MQTT桥接节点
├── include/
│   └── auto_simulation/
│       ├── simulation_node.hpp
│       └── mqtt_bridge.hpp
├── config/                      # 配置文件
│   └── simulation_params.yaml
└── launch/
    └── auto_driving.launch.xml  # 系统启动文件
```

## 主要功能

### 1. simulation_node
**功能**: 生成仿真环境和地图数据
**输入**: 
- 配置参数（地图大小、障碍物位置等）
**输出**: 
- `/grid_map` (auto_msgs/GridMap): 网格地图
- `/planning_request` (auto_msgs/PlanningRequest): 规划请求

**核心算法**:
- 随机障碍物生成
- 网格地图构建
- 仿真环境可视化

### 2. mqtt_bridge
**功能**: 提供ROS2与MQTT之间的通信桥接
**输入**: 
- ROS2话题数据
- MQTT消息
**输出**: 
- MQTT话题发布
- ROS2话题发布

**支持的MQTT话题**:
- `auto_driving/planning_path`: 规划路径
- `auto_driving/command/request_planning`: 规划请求
- `auto_driving/status`: 系统状态
- `auto_driving/vehicle_state`: 车辆状态

## 节点接口

### simulation_node
**发布话题**:
- `/grid_map` (auto_msgs/GridMap): 仿真地图
- `/planning_request` (auto_msgs/PlanningRequest): 规划请求
- `/simulation_status` (std_msgs/String): 仿真状态

**订阅话题**:
- `/reset_simulation` (std_msgs/Empty): 重置仿真

**参数**:
- `map_width`: 地图宽度 (默认: 100)
- `map_height`: 地图高度 (默认: 100)
- `resolution`: 地图分辨率 (默认: 0.1)
- `obstacle_count`: 障碍物数量 (默认: 10)
- `update_rate`: 更新频率 (默认: 10Hz)

### mqtt_bridge_node
**订阅话题**:
- `/planning_path` (auto_msgs/PlanningPath): 规划结果
- `/control_cmd` (geometry_msgs/Twist): 控制指令
- `/vehicle_state` (geometry_msgs/PoseStamped): 车辆状态

**发布话题**:
- `/planning_request` (auto_msgs/PlanningRequest): 来自MQTT的规划请求
- `/external_commands` (std_msgs/String): 外部指令

**参数**:
- `mqtt_host`: MQTT服务器地址 (默认: "localhost")
- `mqtt_port`: MQTT端口 (默认: 1883)
- `client_id`: 客户端ID (默认: "auto_driving_bridge")

## 仿真环境生成

### 地图生成算法
```cpp
class MapGenerator {
public:
    GridMap generateMap(int width, int height, double resolution) {
        GridMap map;
        map.width = width;
        map.height = height;
        map.resolution = resolution;
        
        // 初始化为空闲空间
        map.data.resize(width * height, 0);
        
        // 添加边界
        addBoundaries(map);
        
        // 添加随机障碍物
        addRandomObstacles(map);
        
        return map;
    }
    
private:
    void addBoundaries(GridMap& map);
    void addRandomObstacles(GridMap& map);
};
```

### 障碍物生成策略
1. **随机分布**: 在地图中随机放置障碍物
2. **聚类分布**: 障碍物成群分布
3. **结构化障碍物**: 墙壁、建筑物等
4. **动态障碍物**: 移动的障碍物

## MQTT通信协议

### 消息格式
```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "message_type": "planning_request",
  "data": {
    "start": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "goal": {"x": 10.0, "y": 10.0, "theta": 0.0},
    "planner_type": "astar"
  }
}
```

### 支持的消息类型
- `planning_request`: 路径规划请求
- `planning_path`: 规划路径结果
- `vehicle_state`: 车辆状态更新
- `system_status`: 系统状态信息
- `emergency_stop`: 紧急停车指令

## 测试方法

### 单独测试仿真节点
```bash
# 启动仿真节点
ros2 run auto_simulation simulation_node

# 检查地图发布
ros2 topic echo /grid_map --once

# 检查规划请求
ros2 topic echo /planning_request --once

# 重置仿真
ros2 topic pub /reset_simulation std_msgs/msg/Empty "{}"
```

### 单独测试MQTT桥接
```bash
# 启动MQTT服务器
sudo systemctl start mosquitto

# 启动MQTT桥接
ros2 run auto_simulation mqtt_bridge_node

# 测试MQTT发布
mosquitto_pub -t "auto_driving/command/request_planning" \
  -m '{"start": {"x": 0, "y": 0}, "goal": {"x": 10, "y": 10}}'

# 测试MQTT订阅
mosquitto_sub -t "auto_driving/planning_path"
```

### 集成测试
```bash
# 启动完整系统
ros2 launch auto_simulation auto_driving.launch.xml

# 验证所有节点运行
ros2 node list

# 检查话题通信
ros2 topic list
ros2 topic hz /grid_map
```

### 可视化测试
```bash
# 启动RViz
rviz2 -d config/simulation.rviz

# 查看地图可视化
# 查看障碍物分布
# 查看规划请求可视化
```

## 配置说明

### launch文件参数
```xml
<launch>
  <arg name="planner_type" default="astar" 
       description="规划器类型: astar, hybrid_astar, optimized_astar"/>
  <arg name="use_rviz" default="true" 
       description="是否启动RViz可视化"/>
  <arg name="mqtt_enabled" default="true" 
       description="是否启用MQTT桥接"/>
  <arg name="map_size" default="100" 
       description="地图大小"/>
  <arg name="obstacle_density" default="0.1" 
       description="障碍物密度"/>
</launch>
```

### 配置文件示例
```yaml
# simulation_params.yaml
simulation:
  map:
    width: 100
    height: 100
    resolution: 0.1
    origin:
      x: 0.0
      y: 0.0
      z: 0.0
  
  obstacles:
    count: 10
    min_size: 1
    max_size: 5
    density: 0.1
  
  update_rate: 10.0

mqtt:
  host: "localhost"
  port: 1883
  client_id: "auto_driving_sim"
  topics:
    planning_path: "auto_driving/planning_path"
    planning_request: "auto_driving/command/request_planning"
    vehicle_state: "auto_driving/vehicle_state"
```

## 性能优化

### 地图生成优化
- 使用多线程生成大地图
- 缓存常用地图模板
- 增量更新地图数据

### MQTT通信优化
- 消息压缩
- 连接池管理
- 异步消息处理

## 故障处理

### 常见问题
1. **MQTT连接失败**
   ```bash
   # 检查MQTT服务状态
   sudo systemctl status mosquitto
   
   # 重启MQTT服务
   sudo systemctl restart mosquitto
   ```

2. **地图生成失败**
   ```bash
   # 检查参数配置
   ros2 param list /simulation_node
   
   # 重置仿真
   ros2 topic pub /reset_simulation std_msgs/msg/Empty "{}"
   ```

3. **可视化异常**
   ```bash
   # 检查RViz配置
   rviz2 -d config/simulation.rviz
   
   # 重新加载配置
   ```

## 依赖关系
- **输入依赖**: auto_msgs
- **输出依赖**: 为其他所有模块提供仿真环境
- **外部依赖**: 
  - MQTT库 (paho-mqtt)
  - RViz2
  - FastDDS

## 扩展功能
- 支持多车辆仿真
- 动态环境变化
- 传感器噪声模拟
- 天气条件仿真
- 交通规则仿真 
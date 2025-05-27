# auto_msgs 模块文档

## 模块概述
auto_msgs 是系统的消息定义模块，定义了各个模块之间通信使用的自定义消息类型。

## 模块结构
```
auto_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── GridMap.msg          # 网格地图消息
│   ├── PathPoint.msg        # 路径点消息
│   ├── PlanningPath.msg     # 规划路径消息
│   └── PlanningRequest.msg  # 规划请求消息
├── include/
└── src/
```

## 消息定义

### 1. GridMap.msg
**功能**: 表示二维网格地图
**输入**: 无（消息定义）
**输出**: 网格地图数据结构

```msg
std_msgs/Header header
uint32 width                    # 地图宽度（格子数）
uint32 height                   # 地图高度（格子数）
float64 resolution              # 分辨率（米/格子）
geometry_msgs/Pose origin       # 地图原点位置
int8[] data                     # 地图数据：-1=未知，0=空闲，100=占用
```

**使用场景**:
- 仿真模块发布地图数据
- 感知模块接收地图进行障碍物检测
- 规划模块使用地图进行路径规划

### 2. PathPoint.msg
**功能**: 表示路径中的单个点
**输入**: 无（消息定义）
**输出**: 路径点数据结构

```msg
geometry_msgs/PoseStamped pose  # 位置和朝向
float64 velocity               # 期望速度
float64 curvature             # 曲率
```

**字段说明**:
- `pose`: 包含位置(x,y,z)和朝向(四元数)
- `velocity`: 该点的期望行驶速度(m/s)
- `curvature`: 路径在该点的曲率(1/m)

### 3. PlanningPath.msg
**功能**: 表示完整的规划路径
**输入**: 无（消息定义）
**输出**: 路径数据结构

```msg
std_msgs/Header header
PathPoint[] points             # 路径点序列
float64 total_length          # 路径总长度
string planner_type           # 使用的规划器类型
```

**使用场景**:
- 规划模块发布规划结果
- 控制模块接收路径进行跟踪
- 可视化模块显示规划路径

### 4. PlanningRequest.msg
**功能**: 表示路径规划请求
**输入**: 无（消息定义）
**输出**: 规划请求数据结构

```msg
std_msgs/Header header
geometry_msgs/PoseStamped start       # 起点
geometry_msgs/PoseStamped goal        # 终点
string planner_type                   # 规划器类型
bool consider_kinematic               # 是否考虑运动学约束
```

**规划器类型**:
- `"astar"`: 基础A*算法
- `"hybrid_astar"`: Hybrid A*算法
- `"optimized_astar"`: 优化A*算法

## 消息关系图

```
PlanningRequest ──┐
                  ├──▶ PathPlanner ──▶ PlanningPath
GridMap ──────────┘                        │
                                           ▼
                                    Controller
```

## 测试方法

### 单独测试
```bash
# 编译消息包
colcon build --packages-select auto_msgs

# 检查消息定义
ros2 interface show auto_msgs/msg/GridMap
ros2 interface show auto_msgs/msg/PathPoint
ros2 interface show auto_msgs/msg/PlanningPath
ros2 interface show auto_msgs/msg/PlanningRequest
```

### 功能验证
```bash
# 发布测试消息
ros2 topic pub /test_gridmap auto_msgs/msg/GridMap \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, 
    width: 100, height: 100, resolution: 0.1, 
    origin: {position: {x: 0.0, y: 0.0, z: 0.0}}, 
    data: [0, 0, 0]}"

# 监听消息
ros2 topic echo /test_gridmap

# 测试规划请求
ros2 topic pub /test_request auto_msgs/msg/PlanningRequest \
  "{header: {frame_id: 'map'}, 
    start: {pose: {position: {x: 0.0, y: 0.0}}}, 
    goal: {pose: {position: {x: 10.0, y: 10.0}}}, 
    planner_type: 'astar', 
    consider_kinematic: false}"
```

### 消息验证脚本
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from auto_msgs.msg import GridMap, PlanningRequest, PlanningPath

class MessageValidator(Node):
    def __init__(self):
        super().__init__('message_validator')
        
        # 测试GridMap消息
        self.test_gridmap()
        
        # 测试PlanningRequest消息
        self.test_planning_request()
    
    def test_gridmap(self):
        msg = GridMap()
        msg.header.frame_id = 'map'
        msg.width = 100
        msg.height = 100
        msg.resolution = 0.1
        msg.data = [0] * (100 * 100)
        
        self.get_logger().info('GridMap message created successfully')
    
    def test_planning_request(self):
        msg = PlanningRequest()
        msg.header.frame_id = 'map'
        msg.planner_type = 'astar'
        msg.consider_kinematic = False
        
        self.get_logger().info('PlanningRequest message created successfully')

if __name__ == '__main__':
    rclpy.init()
    validator = MessageValidator()
    rclpy.spin_once(validator)
    rclpy.shutdown()
```

## 性能考虑

### 消息大小优化
- **GridMap**: 大地图时数据量较大，考虑压缩
- **PlanningPath**: 路径点数量影响消息大小
- **序列化效率**: 使用ROS2原生序列化

### 内存使用
- GridMap数据使用int8数组，内存效率高
- 避免不必要的数据复制
- 合理设置消息队列大小

## 依赖关系
- **输入依赖**: std_msgs, geometry_msgs, nav_msgs
- **输出依赖**: 被所有其他模块使用
- **编译依赖**: ROS2 message generation

## 版本兼容性
- ROS2 Humble及以上版本
- 向后兼容性保证
- 消息格式变更需要版本号管理

## 最佳实践
1. **消息设计**: 保持消息结构简洁明确
2. **字段命名**: 使用描述性的字段名
3. **数据类型**: 选择合适的数据类型以节省带宽
4. **文档更新**: 消息变更时及时更新文档 
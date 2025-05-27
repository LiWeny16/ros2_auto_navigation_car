# auto_perception 模块文档

## 模块概述
auto_perception 是感知模块，负责从网格地图中检测和跟踪障碍物，生成障碍物的三维表示。

## 模块结构
```
auto_perception/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── perception_node.cpp     # 主感知节点
│   └── object_detector.cpp     # 障碍物检测器
└── include/
    └── auto_perception/
        ├── perception_node.hpp
        └── object_detector.hpp
```

## 主要功能

### 1. perception_node
**功能**: 感知系统的主节点，协调各个感知组件
**输入**: 
- `/grid_map` (auto_msgs/GridMap): 网格地图数据
**输出**: 
- `/detected_objects` (visualization_msgs/MarkerArray): 检测到的障碍物
- `/obstacle_map` (nav_msgs/OccupancyGrid): 障碍物占用栅格

### 2. object_detector
**功能**: 从网格地图中检测障碍物
**输入**: 
- 网格地图数据
**输出**: 
- 障碍物位置和尺寸信息
- 障碍物分类结果

**核心算法**:
- 连通域分析
- 障碍物聚类
- 边界框计算
- 障碍物分类

## 算法详解

### 障碍物检测流程
```cpp
class ObjectDetector {
public:
    std::vector<DetectedObject> detectObjects(const GridMap& map) {
        // 1. 预处理
        auto binary_map = preprocessMap(map);
        
        // 2. 连通域分析
        auto components = findConnectedComponents(binary_map);
        
        // 3. 特征提取
        std::vector<DetectedObject> objects;
        for (const auto& component : components) {
            auto object = extractFeatures(component);
            if (isValidObject(object)) {
                objects.push_back(object);
            }
        }
        
        // 4. 分类
        classifyObjects(objects);
        
        return objects;
    }
    
private:
    cv::Mat preprocessMap(const GridMap& map);
    std::vector<Component> findConnectedComponents(const cv::Mat& binary_map);
    DetectedObject extractFeatures(const Component& component);
    bool isValidObject(const DetectedObject& object);
    void classifyObjects(std::vector<DetectedObject>& objects);
};
```

### 1. 预处理阶段
**目标**: 提高检测质量，减少噪声
**步骤**:
1. **阈值化**: 将占用概率转换为二值图像
2. **形态学操作**: 去除小噪声，填补空洞
3. **滤波**: 高斯滤波平滑边缘

```cpp
cv::Mat ObjectDetector::preprocessMap(const GridMap& map) {
    cv::Mat binary_map(map.height, map.width, CV_8UC1);
    
    // 阈值化
    for (int i = 0; i < map.data.size(); ++i) {
        binary_map.data[i] = (map.data[i] > 50) ? 255 : 0;
    }
    
    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary_map, binary_map, cv::MORPH_CLOSE, kernel);
    
    return binary_map;
}
```

### 2. 连通域分析
**目标**: 找到所有连通的障碍物区域
**算法**: 使用OpenCV的连通域分析

```cpp
std::vector<Component> ObjectDetector::findConnectedComponents(const cv::Mat& binary_map) {
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary_map, labels, stats, centroids);
    
    std::vector<Component> components;
    for (int i = 1; i < num_labels; ++i) {  // 跳过背景标签0
        Component comp;
        comp.label = i;
        comp.area = stats.at<int>(i, cv::CC_STAT_AREA);
        comp.bbox = cv::Rect(
            stats.at<int>(i, cv::CC_STAT_LEFT),
            stats.at<int>(i, cv::CC_STAT_TOP),
            stats.at<int>(i, cv::CC_STAT_WIDTH),
            stats.at<int>(i, cv::CC_STAT_HEIGHT)
        );
        comp.centroid = cv::Point2f(
            centroids.at<double>(i, 0),
            centroids.at<double>(i, 1)
        );
        components.push_back(comp);
    }
    
    return components;
}
```

### 3. 特征提取
**目标**: 计算每个障碍物的几何特征
**特征**:
- 位置 (x, y)
- 尺寸 (width, height)
- 面积
- 周长
- 形状特征 (长宽比、圆度等)

```cpp
DetectedObject ObjectDetector::extractFeatures(const Component& component) {
    DetectedObject object;
    
    // 基本几何特征
    object.position.x = component.centroid.x * resolution_;
    object.position.y = component.centroid.y * resolution_;
    object.width = component.bbox.width * resolution_;
    object.height = component.bbox.height * resolution_;
    object.area = component.area * resolution_ * resolution_;
    
    // 形状特征
    object.aspect_ratio = static_cast<double>(component.bbox.width) / component.bbox.height;
    object.circularity = calculateCircularity(component);
    
    // 边界框
    object.bounding_box = component.bbox;
    
    return object;
}
```

### 4. 障碍物分类
**目标**: 根据特征对障碍物进行分类
**类别**:
- 静态障碍物 (建筑物、墙壁)
- 动态障碍物 (车辆、行人)
- 未知障碍物

```cpp
void ObjectDetector::classifyObjects(std::vector<DetectedObject>& objects) {
    for (auto& object : objects) {
        if (object.area > large_object_threshold_) {
            object.type = ObjectType::STATIC_LARGE;
        } else if (object.aspect_ratio > vehicle_aspect_ratio_threshold_) {
            object.type = ObjectType::VEHICLE;
        } else if (object.circularity > circular_threshold_) {
            object.type = ObjectType::PEDESTRIAN;
        } else {
            object.type = ObjectType::UNKNOWN;
        }
        
        // 设置置信度
        object.confidence = calculateConfidence(object);
    }
}
```

## 节点接口

### perception_node
**订阅话题**:
- `/grid_map` (auto_msgs/GridMap): 输入地图

**发布话题**:
- `/detected_objects` (visualization_msgs/MarkerArray): 检测结果
- `/obstacle_map` (nav_msgs/OccupancyGrid): 障碍物地图
- `/perception_status` (std_msgs/String): 感知状态

**参数**:
- `min_obstacle_size`: 最小障碍物尺寸 (默认: 0.2m)
- `max_obstacle_size`: 最大障碍物尺寸 (默认: 5.0m)
- `detection_threshold`: 检测阈值 (默认: 50)
- `update_rate`: 更新频率 (默认: 10Hz)

## 数据结构

### DetectedObject
```cpp
struct DetectedObject {
    geometry_msgs::Point position;      // 位置
    double width, height;               // 尺寸
    double area;                        // 面积
    ObjectType type;                    // 类型
    double confidence;                  // 置信度
    cv::Rect bounding_box;             // 边界框
    double aspect_ratio;               // 长宽比
    double circularity;                // 圆度
    ros::Time timestamp;               // 时间戳
};

enum class ObjectType {
    UNKNOWN,
    STATIC_LARGE,
    STATIC_SMALL,
    VEHICLE,
    PEDESTRIAN
};
```

## 测试方法

### 单独测试感知节点
```bash
# 启动感知节点
ros2 run auto_perception perception_node

# 发布测试地图
ros2 bag play test_map_data.bag

# 检查检测结果
ros2 topic echo /detected_objects
ros2 topic echo /obstacle_map

# 监控处理频率
ros2 topic hz /detected_objects
```

### 可视化测试
```bash
# 启动RViz
rviz2 -d config/perception_test.rviz

# 添加显示项目:
# - MarkerArray: /detected_objects
# - OccupancyGrid: /obstacle_map
# - GridMap: /grid_map
```

### 性能测试
```bash
# 测试处理延迟
ros2 run auto_perception test_latency

# 测试内存使用
top -p $(pgrep perception_node)

# 测试不同地图大小的性能
ros2 run auto_perception benchmark_test
```

### 精度测试
```bash
# 运行精度测试
ros2 run auto_perception accuracy_test

# 生成测试报告
python3 scripts/generate_accuracy_report.py
```

## 性能优化

### 算法优化
1. **多线程处理**: 并行处理不同区域
2. **增量更新**: 只处理变化的区域
3. **空间索引**: 使用四叉树加速查询
4. **内存池**: 减少内存分配开销

### 参数调优
```yaml
# perception_params.yaml
perception:
  detection:
    threshold: 50
    min_size: 0.2
    max_size: 5.0
  
  preprocessing:
    gaussian_kernel_size: 3
    morphology_kernel_size: 3
  
  classification:
    vehicle_aspect_ratio: 2.0
    circular_threshold: 0.8
    large_object_threshold: 10.0
  
  performance:
    max_processing_time: 0.1  # 100ms
    memory_limit: 500  # 500MB
```

## 质量指标

### 检测性能
- **检测精度**: >95%
- **误检率**: <5%
- **漏检率**: <3%
- **处理延迟**: <100ms

### 系统性能
- **内存使用**: <500MB
- **CPU使用**: <30%
- **处理频率**: 10Hz

## 故障排除

### 常见问题
1. **检测精度低**
   - 调整检测阈值
   - 优化预处理参数
   - 检查地图质量

2. **处理延迟高**
   - 减少地图分辨率
   - 启用多线程处理
   - 优化算法参数

3. **内存使用过高**
   - 限制处理区域大小
   - 使用内存池
   - 及时释放资源

### 调试工具
```bash
# 查看检测统计
ros2 topic echo /perception_status

# 可视化检测过程
ros2 run auto_perception debug_visualizer

# 性能分析
ros2 run auto_perception profiler
```

## 依赖关系
- **输入依赖**: auto_msgs, auto_simulation
- **输出依赖**: auto_planning (提供障碍物信息)
- **外部依赖**: 
  - OpenCV
  - PCL (可选)
  - Eigen3

## 扩展功能
- 支持3D点云处理
- 多传感器融合
- 目标跟踪
- 运动预测
- 语义分割 
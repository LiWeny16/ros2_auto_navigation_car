#include "carla_integration/carla_sensor_adapter.hpp"
#include <rclcpp/rclcpp.hpp>

namespace carla_integration {

CarlaSensorAdapter::CarlaSensorAdapter() 
    : Node("carla_sensor_adapter") {
    
    // 声明参数
    this->declare_parameter("enable_lidar_filter", true);
    this->declare_parameter("lidar_range_min", 0.5);
    this->declare_parameter("lidar_range_max", 100.0);
    this->declare_parameter("vehicle_frame", "ego_vehicle");
    this->declare_parameter("lidar_frame", "ego_vehicle/lidar");
    this->declare_parameter("camera_frame", "ego_vehicle/camera");
    this->declare_parameter("pose_publish_rate", 50.0);
    
    // 获取参数
    enable_lidar_filter_ = this->get_parameter("enable_lidar_filter").as_bool();
    lidar_range_min_ = this->get_parameter("lidar_range_min").as_double();
    lidar_range_max_ = this->get_parameter("lidar_range_max").as_double();
    vehicle_frame_ = this->get_parameter("vehicle_frame").as_string();
    lidar_frame_ = this->get_parameter("lidar_frame").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    double pose_rate = this->get_parameter("pose_publish_rate").as_double();
    
    RCLCPP_INFO(this->get_logger(), "CARLA Sensor Adapter initialized");
    RCLCPP_INFO(this->get_logger(), "LiDAR filter enabled: %s", enable_lidar_filter_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "LiDAR range: %.1f - %.1f m", lidar_range_min_, lidar_range_max_);
    
    // CARLA 传感器数据订阅者
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/carla/ego_vehicle/lidar", 10,
        std::bind(&CarlaSensorAdapter::lidarCallback, this, std::placeholders::_1));
    
    camera_rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/carla/ego_vehicle/camera/rgb/image_color", 10,
        std::bind(&CarlaSensorAdapter::cameraCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/carla/ego_vehicle/imu", 10,
        std::bind(&CarlaSensorAdapter::imuCallback, this, std::placeholders::_1));
    
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/carla/ego_vehicle/gnss", 10,
        std::bind(&CarlaSensorAdapter::gnssCallback, this, std::placeholders::_1));
    
    vehicle_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/vehicle_status", 10,
        std::bind(&CarlaSensorAdapter::vehicleStatusCallback, this, std::placeholders::_1));
    
    // 标准传感器数据发布者 (给感知模块)
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensors/lidar/points", 10);
    
    camera_rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/sensors/camera/image_raw", 10);
    
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/sensors/imu/data", 10);
    
    gnss_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/sensors/gnss/fix", 10);
    
    vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/vehicle/pose", 10);
    
    // 创建定时器发布车辆位姿
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / pose_rate));
    pose_timer_ = this->create_wall_timer(
        timer_period, std::bind(&CarlaSensorAdapter::publishVehiclePose, this));
}

void CarlaSensorAdapter::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received LiDAR data");
    
    // 处理点云数据
    auto processed_cloud = processPointCloud(*msg);
    
    // 更新帧ID
    processed_cloud.header.frame_id = lidar_frame_;
    
    // 发布处理后的点云
    lidar_pub_->publish(processed_cloud);
}

void CarlaSensorAdapter::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received camera image");
    
    // 更新帧ID并发布
    auto image_msg = *msg;
    image_msg.header.frame_id = camera_frame_;
    camera_rgb_pub_->publish(image_msg);
}

void CarlaSensorAdapter::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received IMU data");
    
    // 更新帧ID并发布
    auto imu_msg = *msg;
    imu_msg.header.frame_id = vehicle_frame_;
    imu_pub_->publish(imu_msg);
}

void CarlaSensorAdapter::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received GNSS data");
    
    // 直接转发GNSS数据
    gnss_pub_->publish(*msg);
}

void CarlaSensorAdapter::vehicleStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received vehicle status: %s", msg->data.c_str());
    
    // 缓存车辆状态用于位姿发布
    current_vehicle_status_ = msg;
}

sensor_msgs::msg::PointCloud2 CarlaSensorAdapter::processPointCloud(const sensor_msgs::msg::PointCloud2& cloud_msg) {
    if (!enable_lidar_filter_) {
        return cloud_msg;
    }
    
    // 简化版本的点云处理 - 暂时不使用 PCL
    // 这里可以添加基本的距离滤波等处理
    RCLCPP_DEBUG(this->get_logger(), "Processing point cloud (simplified version)");
    
    return cloud_msg;
}

void CarlaSensorAdapter::publishVehiclePose() {
    if (!current_vehicle_status_) {
        return;
    }
    
    // 创建位姿消息
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    
    // 从车辆状态中提取位置和方向信息
    // 简化版本：使用固定值作为示例
    pose_msg.pose.position.x = 0.0;  // 可以从vehicle_status解析
    pose_msg.pose.position.y = 0.0;  
    pose_msg.pose.position.z = 0.0;  
    
    // 方向信息（四元数）
    pose_msg.pose.orientation.w = 1.0;  
    pose_msg.pose.orientation.x = 0.0;  
    pose_msg.pose.orientation.y = 0.0;  
    pose_msg.pose.orientation.z = 0.0;
    
    vehicle_pose_pub_->publish(pose_msg);
}

} // namespace carla_integration

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<carla_integration::CarlaSensorAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

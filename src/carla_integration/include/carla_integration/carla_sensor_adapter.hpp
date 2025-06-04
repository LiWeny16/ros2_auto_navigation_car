#ifndef CARLA_SENSOR_ADAPTER_HPP_
#define CARLA_SENSOR_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

// PCL (commented out for now)
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

// CARLA messages (commented out for now)
// #include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

namespace carla_integration {

/**
 * @brief CARLA 传感器数据适配器
 * 
 * 该类负责接收 CARLA 传感器数据并转发给感知模块
 * 支持的传感器：
 * 1. LiDAR 点云数据
 * 2. 摄像头图像数据  
 * 3. IMU 数据
 * 4. GNSS 数据
 * 5. 车辆状态信息
 */
class CarlaSensorAdapter : public rclcpp::Node {
public:
    CarlaSensorAdapter();

private:
    /**
     * @brief LiDAR 点云数据回调函数
     * @param msg 点云消息
     */
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    /**
     * @brief 摄像头图像回调函数
     * @param msg 图像消息
     */
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief IMU 数据回调函数
     * @param msg IMU 消息
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    /**
     * @brief GNSS 数据回调函数
     * @param msg GNSS 消息
     */
    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    
    /**
     * @brief 车辆状态回调函数
     * @param msg 车辆状态消息
     */
    void vehicleStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    
    /**
     * @brief 处理点云数据，进行滤波和预处理
     * @param cloud_msg 原始点云消息
     * @return 处理后的点云消息
     */
    sensor_msgs::msg::PointCloud2 processPointCloud(const sensor_msgs::msg::PointCloud2& cloud_msg);
    
    /**
     * @brief 发布车辆位姿信息
     */
    void publishVehiclePose();

    // CARLA 传感器数据订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_status_sub_;
    
    // 标准传感器数据发布者 (给感知模块)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr pose_timer_;
    
    // 缓存的车辆状态
    std_msgs::msg::String::SharedPtr current_vehicle_status_;
    
    // 参数
    bool enable_lidar_filter_;
    double lidar_range_min_;
    double lidar_range_max_;
    std::string vehicle_frame_;
    std::string lidar_frame_;
    std::string camera_frame_;
};

} // namespace carla_integration

#endif // CARLA_SENSOR_ADAPTER_HPP_

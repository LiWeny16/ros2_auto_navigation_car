#include <gtest/gtest.h>
#include <memory>
#include <cmath>       // 添加数学函数的头文件
#include "auto_control/pure_pursuit_controller.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/path_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

// 定义 M_PI (如果未定义)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 创建一个简单的直线路径进行测试
auto_msgs::msg::PlanningPath createStraightPath(double start_x, double start_y, double end_x, double end_y, int num_points = 10) {
    auto_msgs::msg::PlanningPath path;
    path.header.frame_id = "map";
    path.planner_type = "test";
    path.total_distance = std::hypot(end_x - start_x, end_y - start_y);
    
    // 在起点和终点之间均匀插入点
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        double x = start_x + t * (end_x - start_x);
        double y = start_y + t * (end_y - start_y);
        
        auto_msgs::msg::PathPoint point;
        point.pose.position.x = x;
        point.pose.position.y = y;
        point.pose.position.z = 0.0;
        
        // 计算方向（假设路径方向沿着直线）
        double yaw = std::atan2(end_y - start_y, end_x - start_x);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        point.pose.orientation.x = q.x();
        point.pose.orientation.y = q.y();
        point.pose.orientation.z = q.z();
        point.pose.orientation.w = q.w();
        
        // 设置其他属性
        point.velocity = 2.0;  // 假设恒定速度
        point.steering_angle = 0.0;  // 直线路径，转向角为0
        point.curvature = 0.0;  // 直线路径，曲率为0
        
        path.points.push_back(point);
    }
    
    return path;
}

// 创建一个简单的圆弧路径进行测试
auto_msgs::msg::PlanningPath createCurvedPath(double center_x, double center_y, double radius, double start_angle, double end_angle, int num_points = 10) {
    auto_msgs::msg::PlanningPath path;
    path.header.frame_id = "map";
    path.planner_type = "test";
    
    // 计算弧长
    double arc_length = radius * std::abs(end_angle - start_angle);
    path.total_distance = arc_length;
    
    // 在起始角度和结束角度之间均匀插入点
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        double angle = start_angle + t * (end_angle - start_angle);
        double x = center_x + radius * std::cos(angle);
        double y = center_y + radius * std::sin(angle);
        
        auto_msgs::msg::PathPoint point;
        point.pose.position.x = x;
        point.pose.position.y = y;
        point.pose.position.z = 0.0;
        
        // 计算切线方向（垂直于半径方向）
        double yaw = angle + M_PI / 2.0;
        if (end_angle < start_angle) {
            yaw = angle - M_PI / 2.0;  // 顺时针方向
        }
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        point.pose.orientation.x = q.x();
        point.pose.orientation.y = q.y();
        point.pose.orientation.z = q.z();
        point.pose.orientation.w = q.w();
        
        // 设置其他属性
        point.velocity = 2.0;  // 假设恒定速度
        point.steering_angle = 0.0;  // 将在控制器中计算
        point.curvature = 1.0 / radius;  // 曲率为半径的倒数
        
        path.points.push_back(point);
    }
    
    return path;
}

// 纯跟踪控制器基本测试
TEST(ControllerTest, PurePursuitBasicTest) {
    // 创建控制器
    auto_control::PurePursuitController controller;
    controller.setLookaheadDistance(2.0);
    controller.setMaxVelocity(5.0);
    
    // 创建一条简单的直线路径
    auto path = createStraightPath(0.0, 0.0, 10.0, 0.0, 20);
    controller.setPath(path);
    
    // 测试控制器状态
    EXPECT_TRUE(controller.hasPath());
    EXPECT_FALSE(controller.reachedGoal());
    
    // 创建当前位置（在路径起点附近，但有一点偏移）
    geometry_msgs::msg::Pose current_pose;
    current_pose.position.x = 0.0;
    current_pose.position.y = 0.5;  // 稍微偏离路径
    current_pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);  // 朝向x轴正方向
    current_pose.orientation.x = q.x();
    current_pose.orientation.y = q.y();
    current_pose.orientation.z = q.z();
    current_pose.orientation.w = q.w();
    
    // 创建当前速度（初始静止）
    geometry_msgs::msg::Twist current_velocity;
    current_velocity.linear.x = 0.0;
    current_velocity.linear.y = 0.0;
    current_velocity.angular.z = 0.0;
    
    // 计算控制命令
    auto cmd = controller.calculateControl(current_pose, current_velocity);
    
    // 验证控制命令
    // 由于偏离路径在左侧，应该向右转向（负转向角）
    EXPECT_LT(cmd.steering_angle, 0.0);
    // 应该有油门，没有制动
    EXPECT_GT(cmd.throttle, 0.0);
    EXPECT_EQ(cmd.brake, 0.0);
}

// 测试接近终点时的行为
TEST(ControllerTest, ApproachGoalTest) {
    // 创建控制器
    auto_control::PurePursuitController controller;
    controller.setLookaheadDistance(2.0);
    controller.setMaxVelocity(5.0);
    
    // 创建一条简单的直线路径
    auto path = createStraightPath(0.0, 0.0, 10.0, 0.0, 20);
    controller.setPath(path);
    
    // 创建当前位置（非常接近终点）
    geometry_msgs::msg::Pose current_pose;
    current_pose.position.x = 8.5;  // 明确设置在终点(10.0, 0.0)之前1.5米，确保在制动区间内
    current_pose.position.y = 0.0;
    current_pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);  // 朝向x轴正方向
    current_pose.orientation.x = q.x();
    current_pose.orientation.y = q.y();
    current_pose.orientation.z = q.z();
    current_pose.orientation.w = q.w();
    
    // 创建当前速度（已经在移动）
    geometry_msgs::msg::Twist current_velocity;
    current_velocity.linear.x = 2.0;
    current_velocity.linear.y = 0.0;
    current_velocity.angular.z = 0.0;
    
    // 计算控制命令
    auto cmd = controller.calculateControl(current_pose, current_velocity);
    
    // 验证控制命令
    // 应该有制动，因为接近终点
    EXPECT_GT(cmd.brake, 0.0);
}

// 测试转弯路径上的控制
TEST(ControllerTest, CurvedPathTest) {
    // 创建控制器
    auto_control::PurePursuitController controller;
    controller.setLookaheadDistance(3.0);
    controller.setMaxVelocity(5.0);
    
    // 创建一条圆弧路径（顺时针90度转弯）
    auto path = createCurvedPath(10.0, 0.0, 10.0, M_PI, M_PI / 2.0, 20);
    controller.setPath(path);
    
    // 创建当前位置（在圆弧起点）
    geometry_msgs::msg::Pose current_pose;
    current_pose.position.x = 0.0;  // 圆心(10, 0)，半径10，起始角度π，所以位置是(0, 0)
    current_pose.position.y = 0.0;
    current_pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, M_PI / 2.0);  // 朝向y轴正方向
    current_pose.orientation.x = q.x();
    current_pose.orientation.y = q.y();
    current_pose.orientation.z = q.z();
    current_pose.orientation.w = q.w();
    
    // 创建当前速度
    geometry_msgs::msg::Twist current_velocity;
    current_velocity.linear.x = 0.0;
    current_velocity.linear.y = 2.0;  // 沿y轴正方向移动
    current_velocity.angular.z = 0.0;
    
    // 计算控制命令
    auto cmd = controller.calculateControl(current_pose, current_velocity);
    
    // 验证控制命令
    // 应该向右转向（负转向角）来跟随顺时针圆弧
    EXPECT_LT(cmd.steering_angle, 0.0);
    // 应该有油门，因为我们刚开始沿着新路径
    EXPECT_GT(cmd.throttle, 0.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

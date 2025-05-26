#include <gtest/gtest.h>
#include <memory>
#include "auto_planning/a_star_planner.hpp"
#include "auto_planning/hybrid_a_star_planner.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// 创建一个小型测试地图
auto_msgs::msg::GridMap createTestMap(int width, int height, double resolution) {
    auto_msgs::msg::GridMap map;
    map.header.frame_id = "map";
    map.width = width;
    map.height = height;
    map.resolution = resolution;
    map.data.resize(width * height, 0);  // 初始化为全空闲
    map.origin.position.x = 0.0;
    map.origin.position.y = 0.0;
    map.origin.position.z = 0.0;
    map.origin.orientation.w = 1.0;
    return map;
}

// 在地图上添加障碍物
void addObstacle(auto_msgs::msg::GridMap& map, int x, int y, int size) {
    for (int i = x - size/2; i < x + size/2; ++i) {
        for (int j = y - size/2; j < y + size/2; ++j) {
            if (i >= 0 && i < map.width && j >= 0 && j < map.height) {
                map.data[j * map.width + i] = 100;  // 设置为占用
            }
        }
    }
}

// 创建起点和终点
geometry_msgs::msg::PoseStamped createPose(double x, double y, double yaw = 0.0) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    
    // 设置方向
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    return pose;
}

// A*规划器测试
TEST(PlannerTest, AStarBasicTest) {
    // 创建一个20x20的地图，分辨率为0.5米/格
    auto map = createTestMap(20, 20, 0.5);
    
    // 添加一些障碍物
    addObstacle(map, 10, 10, 4);
    
    // 创建A*规划器
    auto_planning::AStarPlanner planner;
    
    // 创建起点和终点
    auto start = createPose(2.5, 2.5);
    auto goal = createPose(7.5, 7.5);
    
    // 执行规划
    auto path = planner.plan(map, start, goal);
    
    // 验证结果
    EXPECT_FALSE(path.points.empty());
    EXPECT_GT(path.total_distance, 0.0);
    EXPECT_EQ(path.planner_type, "astar");
    
    // 验证路径起点和终点
    if (!path.points.empty()) {
        double start_x = path.points.front().pose.position.x;
        double start_y = path.points.front().pose.position.y;
        double goal_x = path.points.back().pose.position.x;
        double goal_y = path.points.back().pose.position.y;
        
        EXPECT_NEAR(start_x, start.pose.position.x, 0.5);
        EXPECT_NEAR(start_y, start.pose.position.y, 0.5);
        EXPECT_NEAR(goal_x, goal.pose.position.x, 0.5);
        EXPECT_NEAR(goal_y, goal.pose.position.y, 0.5);
    }
}

// Hybrid A*规划器测试
TEST(PlannerTest, HybridAStarBasicTest) {
    // 创建一个20x20的地图，分辨率为0.5米/格
    auto map = createTestMap(20, 20, 0.5);
    
    // 添加一些障碍物
    addObstacle(map, 10, 10, 4);
    
    // 创建Hybrid A*规划器
    auto_planning::HybridAStarPlanner planner;
    
    // 创建起点和终点，包含方向
    auto start = createPose(2.5, 2.5, 0.0);
    auto goal = createPose(7.5, 7.5, 1.57);  // 目标朝向π/2（向上）
    
    // 执行规划
    auto path = planner.plan(map, start, goal);
    
    // 验证结果
    EXPECT_FALSE(path.points.empty());
    EXPECT_GT(path.total_distance, 0.0);
    EXPECT_EQ(path.planner_type, "hybrid_astar");
    
    // 验证路径起点和终点
    if (!path.points.empty()) {
        double start_x = path.points.front().pose.position.x;
        double start_y = path.points.front().pose.position.y;
        double goal_x = path.points.back().pose.position.x;
        double goal_y = path.points.back().pose.position.y;
        
        EXPECT_NEAR(start_x, start.pose.position.x, 0.5);
        EXPECT_NEAR(start_y, start.pose.position.y, 0.5);
        EXPECT_NEAR(goal_x, goal.pose.position.x, 0.5);
        EXPECT_NEAR(goal_y, goal.pose.position.y, 0.5);
        
        // 验证终点方向（简化版，仅检查z分量的符号）
        double goal_qz = path.points.back().pose.orientation.z;
        double expected_qz = goal.pose.orientation.z;
        EXPECT_TRUE((goal_qz > 0 && expected_qz > 0) || (goal_qz < 0 && expected_qz < 0));
    }
}

// 无解情况测试
TEST(PlannerTest, NoSolutionTest) {
    // 创建一个20x20的地图，分辨率为0.5米/格
    auto map = createTestMap(20, 20, 0.5);
    
    // 添加障碍物围墙，将地图分成两半
    for (int i = 0; i < 20; ++i) {
        addObstacle(map, 10, i, 1);
    }
    
    // 创建A*规划器
    auto_planning::AStarPlanner planner;
    
    // 创建起点和终点，分别在障碍物两侧
    auto start = createPose(5.0, 10.0);
    auto goal = createPose(15.0, 10.0);
    
    // 执行规划
    auto path = planner.plan(map, start, goal);
    
    // 验证结果
    EXPECT_TRUE(path.points.empty());  // 应该找不到路径
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

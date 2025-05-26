#include "auto_integration_test/system_test_runner.hpp"
#include <fstream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <random>

namespace auto_integration_test {

using namespace std::chrono_literals;

SystemTestRunner::SystemTestRunner() : Node("system_test_runner"), received_result_(false) {
    // 创建发布者
    map_pub_ = this->create_publisher<auto_msgs::msg::GridMap>("grid_map", 10);
    request_pub_ = this->create_publisher<auto_msgs::msg::PlanningRequest>("planning_request", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("test_status", 10);
    
    // 创建订阅者
    path_sub_ = this->create_subscription<auto_msgs::msg::PlanningPath>(
        "planning_path", 10, std::bind(&SystemTestRunner::planningPathCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "系统测试运行器已启动");
    
    // 等待一秒，确保ROS2节点连接
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void SystemTestRunner::loadTestCases() {
    RCLCPP_INFO(this->get_logger(), "加载测试用例...");
    
    // 测试用例1：简单路径规划 (A*)
    {
        TestCase test;
        test.name = "simple_path_astar";
        test.description = "简单A*路径规划，没有障碍物";
        
        // 设置起点和终点
        test.start.header.frame_id = "map";
        test.start.pose.position.x = 5.0;
        test.start.pose.position.y = 5.0;
        test.start.pose.orientation.w = 1.0;
        
        test.goal.header.frame_id = "map";
        test.goal.pose.position.x = 15.0;
        test.goal.pose.position.y = 15.0;
        test.goal.pose.orientation.w = 1.0;
        
        test.planner_type = "astar";
        test.expect_success = true;
        test.expected_max_time = 0.1;  // 预期最大规划时间（秒）
        
        test_cases_.push_back(test);
    }
    
    // 测试用例2：带障碍物的路径规划 (A*)
    {
        TestCase test;
        test.name = "obstacle_path_astar";
        test.description = "A*路径规划，存在障碍物";
        
        // 设置起点和终点
        test.start.header.frame_id = "map";
        test.start.pose.position.x = 5.0;
        test.start.pose.position.y = 5.0;
        test.start.pose.orientation.w = 1.0;
        
        test.goal.header.frame_id = "map";
        test.goal.pose.position.x = 15.0;
        test.goal.pose.position.y = 15.0;
        test.goal.pose.orientation.w = 1.0;
        
        test.planner_type = "astar";
        test.expect_success = true;
        test.expected_max_time = 0.2;
        
        test_cases_.push_back(test);
    }
    
    // 测试用例3：简单路径规划 (Hybrid A*)
    {
        TestCase test;
        test.name = "simple_path_hybrid_astar";
        test.description = "简单Hybrid A*路径规划，没有障碍物";
        
        // 设置起点和终点（包含方向）
        test.start.header.frame_id = "map";
        test.start.pose.position.x = 5.0;
        test.start.pose.position.y = 5.0;
        test.start.pose.orientation.w = 0.7071;
        test.start.pose.orientation.z = 0.7071;  // 约45度方向
        
        test.goal.header.frame_id = "map";
        test.goal.pose.position.x = 15.0;
        test.goal.pose.position.y = 15.0;
        test.goal.pose.orientation.w = 0.7071;
        test.goal.pose.orientation.z = 0.7071;
        
        test.planner_type = "hybrid_astar";
        test.expect_success = true;
        test.expected_max_time = 0.3;
        
        test_cases_.push_back(test);
    }
    
    // 测试用例4：带障碍物的路径规划 (Hybrid A*)
    {
        TestCase test;
        test.name = "obstacle_path_hybrid_astar";
        test.description = "Hybrid A*路径规划，存在障碍物";
        
        // 设置起点和终点
        test.start.header.frame_id = "map";
        test.start.pose.position.x = 5.0;
        test.start.pose.position.y = 5.0;
        test.start.pose.orientation.w = 0.7071;
        test.start.pose.orientation.z = 0.7071;
        
        test.goal.header.frame_id = "map";
        test.goal.pose.position.x = 15.0;
        test.goal.pose.position.y = 15.0;
        test.goal.pose.orientation.w = 0.7071;
        test.goal.pose.orientation.z = 0.7071;
        
        test.planner_type = "hybrid_astar";
        test.expect_success = true;
        test.expected_max_time = 0.5;
        
        test_cases_.push_back(test);
    }
    
    // 测试用例5：无解路径
    {
        TestCase test;
        test.name = "unsolvable_path";
        test.description = "无法找到路径的情况";
        
        // 设置起点和终点
        test.start.header.frame_id = "map";
        test.start.pose.position.x = 5.0;
        test.start.pose.position.y = 10.0;
        test.start.pose.orientation.w = 1.0;
        
        test.goal.header.frame_id = "map";
        test.goal.pose.position.x = 15.0;
        test.goal.pose.position.y = 10.0;
        test.goal.pose.orientation.w = 1.0;
        
        test.planner_type = "astar";
        test.expect_success = false;  // 预期失败
        test.expected_max_time = 0.5;
        
        test_cases_.push_back(test);
    }
    
    RCLCPP_INFO(this->get_logger(), "已加载 %zu 个测试用例", test_cases_.size());
}

void SystemTestRunner::runAllTests() {
    RCLCPP_INFO(this->get_logger(), "开始运行所有测试...");
    
    for (const auto& test : test_cases_) {
        RCLCPP_INFO(this->get_logger(), "运行测试: %s", test.name.c_str());
        runTest(test);
        
        // 等待一段时间，让系统稳定
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    RCLCPP_INFO(this->get_logger(), "所有测试完成");
    
    // 输出测试结果摘要
    int passed = 0;
    for (const auto& result : test_results_) {
        if (result.success) {
            passed++;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "测试摘要: %d/%zu 通过", passed, test_results_.size());
}

void SystemTestRunner::runTest(const TestCase& test_case) {
    // 重置结果标志
    received_result_ = false;
    
    // 初始化当前测试结果
    current_result_.test_name = test_case.name;
    current_result_.success = false;
    current_result_.failure_reason = "";
    current_result_.planning_time = 0.0;
    current_result_.total_distance = 0.0;
    current_result_.path_points = 0;
    current_result_.control_error_max = 0.0;
    current_result_.control_error_avg = 0.0;
    
    // 发布测试状态
    publishTestStatus("开始测试: " + test_case.name);
    
    // 创建测试地图
    std::vector<std::pair<int, int>> obstacles;
    
    if (test_case.name == "obstacle_path_astar" || test_case.name == "obstacle_path_hybrid_astar") {
        // 添加随机障碍物
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 19);
        
        for (int i = 0; i < 20; ++i) {
            int x = dis(gen);
            int y = dis(gen);
            
            // 避免在起点和终点附近放置障碍物
            if ((std::abs(x * 1.0 - test_case.start.pose.position.x) < 2.0 &&
                 std::abs(y * 1.0 - test_case.start.pose.position.y) < 2.0) ||
                (std::abs(x * 1.0 - test_case.goal.pose.position.x) < 2.0 &&
                 std::abs(y * 1.0 - test_case.goal.pose.position.y) < 2.0)) {
                continue;
            }
            
            obstacles.emplace_back(x, y);
        }
    } else if (test_case.name == "unsolvable_path") {
        // 创建一道障碍墙
        for (int i = 0; i < 20; ++i) {
            obstacles.emplace_back(10, i);
        }
    }
    
    createTestMap(20, 20, 1.0, obstacles);
    
    // 发布地图
    map_pub_->publish(current_map_);
    
    // 创建规划请求
    auto_msgs::msg::PlanningRequest request;
    request.header.stamp = this->now();
    request.header.frame_id = "map";
    request.start = test_case.start;
    request.goal = test_case.goal;
    request.planner_type = test_case.planner_type;
    request.consider_kinematic = (test_case.planner_type == "hybrid_astar");
    
    // 发布规划请求
    request_pub_->publish(request);
    
    // 等待结果（最多5秒）
    bool got_result = waitForResult(std::chrono::seconds(5));
    
    if (!got_result) {
        // 超时
        current_result_.success = false;
        current_result_.failure_reason = "超时，未收到规划结果";
    } else if (latest_path_.points.empty() && test_case.expect_success) {
        // 预期成功但失败了
        current_result_.success = false;
        current_result_.failure_reason = "未能找到路径，但预期应该成功";
    } else if (!latest_path_.points.empty() && !test_case.expect_success) {
        // 预期失败但成功了
        current_result_.success = false;
        current_result_.failure_reason = "找到了路径，但预期应该失败";
    } else if (latest_path_.planning_time > test_case.expected_max_time) {
        // 规划时间超过预期
        current_result_.success = false;
        current_result_.failure_reason = "规划时间超过预期: " + 
            std::to_string(latest_path_.planning_time) + " > " + 
            std::to_string(test_case.expected_max_time);
    } else {
        // 测试通过
        current_result_.success = true;
        
        // 如果有路径，计算控制误差
        if (!latest_path_.points.empty()) {
            auto errors = calculateControlError(latest_path_);
            current_result_.control_error_max = errors.first;
            current_result_.control_error_avg = errors.second;
        }
    }
    
    // 记录其他结果信息
    if (!latest_path_.points.empty()) {
        current_result_.planning_time = latest_path_.planning_time;
        current_result_.total_distance = latest_path_.total_distance;
        current_result_.path_points = latest_path_.points.size();
    }
    
    // 添加到结果列表
    test_results_.push_back(current_result_);
    
    // 输出结果
    if (current_result_.success) {
        RCLCPP_INFO(this->get_logger(), "测试通过: %s", test_case.name.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "测试失败: %s - %s", 
                   test_case.name.c_str(), current_result_.failure_reason.c_str());
    }
    
    // 发布测试状态
    publishTestStatus("完成测试: " + test_case.name + 
                     (current_result_.success ? " (通过)" : " (失败)"));
}

void SystemTestRunner::saveResults(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开文件保存测试结果: %s", filename.c_str());
        return;
    }
    
    // 写入CSV表头
    file << "测试名称,成功,失败原因,规划时间(秒),路径长度(米),路径点数,最大控制误差,平均控制误差\n";
    
    // 写入测试结果
    for (const auto& result : test_results_) {
        file << result.test_name << ","
             << (result.success ? "通过" : "失败") << ","
             << result.failure_reason << ","
             << result.planning_time << ","
             << result.total_distance << ","
             << result.path_points << ","
             << result.control_error_max << ","
             << result.control_error_avg << "\n";
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "测试结果已保存到: %s", filename.c_str());
}

void SystemTestRunner::generatePerformanceBenchmark() {
    RCLCPP_INFO(this->get_logger(), "生成性能基准...");
    
    // 提取A*和Hybrid A*的性能数据
    std::vector<double> astar_times;
    std::vector<double> hybrid_astar_times;
    
    for (const auto& result : test_results_) {
        // 跳过失败的测试
        if (!result.success || result.planning_time <= 0) {
            continue;
        }
        
        if (result.test_name.find("astar") != std::string::npos && 
            result.test_name.find("hybrid") == std::string::npos) {
            astar_times.push_back(result.planning_time);
        } else if (result.test_name.find("hybrid_astar") != std::string::npos) {
            hybrid_astar_times.push_back(result.planning_time);
        }
    }
    
    // 计算平均规划时间
    double astar_avg_time = 0.0;
    double hybrid_astar_avg_time = 0.0;
    
    if (!astar_times.empty()) {
        astar_avg_time = std::accumulate(astar_times.begin(), astar_times.end(), 0.0) / astar_times.size();
    }
    
    if (!hybrid_astar_times.empty()) {
        hybrid_astar_avg_time = std::accumulate(hybrid_astar_times.begin(), hybrid_astar_times.end(), 0.0) / hybrid_astar_times.size();
    }
    
    RCLCPP_INFO(this->get_logger(), "性能基准:");
    RCLCPP_INFO(this->get_logger(), "  A* 平均规划时间: %.4f 秒", astar_avg_time);
    RCLCPP_INFO(this->get_logger(), "  Hybrid A* 平均规划时间: %.4f 秒", hybrid_astar_avg_time);
    
    // 可以再保存到文件或数据库中
}

void SystemTestRunner::createTestMap(int width, int height, double resolution, 
                                   const std::vector<std::pair<int, int>>& obstacles) {
    auto_msgs::msg::GridMap map;
    map.header.frame_id = "map";
    map.header.stamp = this->now();
    map.width = width;
    map.height = height;
    map.resolution = resolution;
    map.data.resize(width * height, 0);  // 初始化为空闲
    
    // 设置地图原点
    map.origin.position.x = 0.0;
    map.origin.position.y = 0.0;
    map.origin.position.z = 0.0;
    map.origin.orientation.w = 1.0;
    
    // 添加障碍物
    for (const auto& obstacle : obstacles) {
        int x = obstacle.first;
        int y = obstacle.second;
        
        if (x >= 0 && x < width && y >= 0 && y < height) {
            map.data[y * width + x] = 100;  // 设置为占用
        }
    }
    
    current_map_ = map;
}

void SystemTestRunner::planningPathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到规划路径，包含 %zu 个点", msg->points.size());
    latest_path_ = *msg;
    received_result_ = true;
}

void SystemTestRunner::publishTestStatus(const std::string& status) {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
}

bool SystemTestRunner::waitForResult(std::chrono::seconds timeout) {
    auto start = std::chrono::steady_clock::now();
    
    while (rclcpp::ok()) {
        if (received_result_) {
            return true;
        }
        
        if (std::chrono::steady_clock::now() - start > timeout) {
            return false;  // 超时
        }
        
        // 非阻塞式等待
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return false;
}

std::pair<double, double> SystemTestRunner::calculateControlError(const auto_msgs::msg::PlanningPath& path) {
    // 简化版控制误差计算，这里我们模拟车辆沿路径行驶并计算跟踪误差
    // 实际系统中应该使用控制模块的实际输出
    
    if (path.points.size() < 2) {
        return {0.0, 0.0};
    }
    
    double max_error = 0.0;
    double total_error = 0.0;
    int count = 0;
    
    // 模拟车辆位置（从起点开始）
    double vehicle_x = path.points[0].pose.position.x;
    double vehicle_y = path.points[0].pose.position.y;
    
    // 遍历路径点计算误差
    for (size_t i = 1; i < path.points.size(); ++i) {
        // 目标点
        double target_x = path.points[i].pose.position.x;
        double target_y = path.points[i].pose.position.y;
        
        // 计算到目标点的方向
        double dx = target_x - vehicle_x;
        double dy = target_y - vehicle_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // 添加一些随机扰动模拟控制误差
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> dis(0.0, 0.05);  // 均值0，标准差0.05的正态分布
        
        double error = std::abs(dis(gen));
        max_error = std::max(max_error, error);
        total_error += error;
        count++;
        
        // 更新车辆位置（假设车辆能够跟随路径，但有一些误差）
        double move_ratio = 0.9;  // 移动比例（模拟控制不完美）
        vehicle_x += dx * move_ratio;
        vehicle_y += dy * move_ratio;
    }
    
    double avg_error = (count > 0) ? (total_error / count) : 0.0;
    return {max_error, avg_error};
}

} // namespace auto_integration_test

#ifndef SYSTEM_TEST_RUNNER_HPP_
#define SYSTEM_TEST_RUNNER_HPP_

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace auto_integration_test {

// 定义测试用例结构
struct TestCase {
    std::string name;
    std::string description;
    geometry_msgs::msg::PoseStamped start;
    geometry_msgs::msg::PoseStamped goal;
    std::string planner_type;
    bool expect_success;
    double expected_max_time;
};

// 定义测试结果结构
struct TestResult {
    std::string test_name;
    bool success;
    std::string failure_reason;
    double planning_time;
    double total_distance;
    int path_points;
    double control_error_max;
    double control_error_avg;
};

// 系统测试运行器类
class SystemTestRunner : public rclcpp::Node {
public:
    SystemTestRunner();
    virtual ~SystemTestRunner() = default;

    // 加载测试用例
    void loadTestCases();
    
    // 运行所有测试
    void runAllTests();
    
    // 运行单个测试
    void runTest(const TestCase& test_case);
    
    // 保存测试结果
    void saveResults(const std::string& filename);
    
    // 获取性能基准
    void generatePerformanceBenchmark();

private:
    // 创建测试地图
    void createTestMap(int width, int height, double resolution, const std::vector<std::pair<int, int>>& obstacles);
    
    // 等待规划结果的回调
    void planningPathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg);
    
    // 发布测试状态
    void publishTestStatus(const std::string& status);
    
    // 等待超时函数
    bool waitForResult(std::chrono::seconds timeout);
    
    // 计算路径跟踪误差
    std::pair<double, double> calculateControlError(const auto_msgs::msg::PlanningPath& path);

    // 测试用例集合
    std::vector<TestCase> test_cases_;
    
    // 测试结果集合
    std::vector<TestResult> test_results_;
    
    // 当前测试结果
    TestResult current_result_;
    
    // 是否收到规划结果
    bool received_result_;
    
    // 最新规划路径
    auto_msgs::msg::PlanningPath latest_path_;
    
    // 发布者和订阅者
    rclcpp::Publisher<auto_msgs::msg::GridMap>::SharedPtr map_pub_;
    rclcpp::Publisher<auto_msgs::msg::PlanningRequest>::SharedPtr request_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_;
    
    // 当前地图
    auto_msgs::msg::GridMap current_map_;
};

} // namespace auto_integration_test

#endif // SYSTEM_TEST_RUNNER_HPP_

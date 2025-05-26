#include "auto_integration_test/system_test_runner.hpp"
#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::string results_file = "test_results.csv";
    if (argc > 1) {
        results_file = argv[1];
    }
    
    auto test_runner = std::make_shared<auto_integration_test::SystemTestRunner>();
    
    test_runner->loadTestCases();
    test_runner->runAllTests();
    test_runner->saveResults(results_file);
    test_runner->generatePerformanceBenchmark();
    
    rclcpp::shutdown();
    return 0;
}

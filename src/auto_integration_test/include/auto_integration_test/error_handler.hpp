#ifndef ERROR_HANDLER_HPP_
#define ERROR_HANDLER_HPP_

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

namespace auto_integration_test {

// 错误严重性级别
enum class ErrorSeverity {
    INFO,       // 信息性消息
    WARNING,    // 警告，可能导致次优性能但不影响功能
    ERROR,      // 错误，可能导致功能降级
    CRITICAL    // 严重错误，可能导致系统失败
};

// 错误来源
enum class ErrorSource {
    PERCEPTION,     // 感知模块
    PLANNING,       // 规划模块
    CONTROL,        // 控制模块
    SIMULATION,     // 仿真模块
    SYSTEM          // 系统级
};

// 错误记录结构
struct ErrorRecord {
    ErrorSeverity severity;
    ErrorSource source;
    std::string message;
    std::string timestamp;
    std::string stacktrace;
    std::string recovery_action;
};

// 错误处理和分析类
class ErrorHandler {
public:
    static ErrorHandler& getInstance() {
        static ErrorHandler instance;
        return instance;
    }
    
    // 注册错误处理回调
    using ErrorCallback = std::function<void(const ErrorRecord&)>;
    void registerErrorCallback(ErrorSeverity severity, ErrorCallback callback);
    
    // 记录错误
    void logError(ErrorSeverity severity, ErrorSource source, 
                 const std::string& message, const std::string& recovery_action = "");
    
    // 获取错误历史
    std::vector<ErrorRecord> getErrorHistory() const;
    
    // 获取特定模块的错误统计
    std::map<ErrorSeverity, int> getErrorStats(ErrorSource source) const;
    
    // 分析系统健康状况
    std::string analyzeSystemHealth() const;
    
    // 生成错误诊断报告
    void generateErrorReport(const std::string& filename) const;
    
private:
    ErrorHandler() {}  // 私有构造函数，确保单例
    ~ErrorHandler() = default;
    
    ErrorHandler(const ErrorHandler&) = delete;
    ErrorHandler& operator=(const ErrorHandler&) = delete;
    
    // 获取当前时间戳
    std::string getCurrentTimestamp() const;
    
    // 获取简化的堆栈跟踪
    std::string getStackTrace() const;
    
    // 错误历史
    std::vector<ErrorRecord> error_history_;
    
    // 错误处理回调
    std::map<ErrorSeverity, std::vector<ErrorCallback>> error_callbacks_;
    
    // 互斥锁，保护共享资源
    mutable std::mutex mutex_;
};

// 便利宏，用于记录错误
#define LOG_INFO(source, message) \
    auto_integration_test::ErrorHandler::getInstance().logError( \
        auto_integration_test::ErrorSeverity::INFO, source, message)

#define LOG_WARNING(source, message, recovery) \
    auto_integration_test::ErrorHandler::getInstance().logError( \
        auto_integration_test::ErrorSeverity::WARNING, source, message, recovery)

#define LOG_ERROR(source, message, recovery) \
    auto_integration_test::ErrorHandler::getInstance().logError( \
        auto_integration_test::ErrorSeverity::ERROR, source, message, recovery)

#define LOG_CRITICAL(source, message, recovery) \
    auto_integration_test::ErrorHandler::getInstance().logError( \
        auto_integration_test::ErrorSeverity::CRITICAL, source, message, recovery)

} // namespace auto_integration_test

#endif // ERROR_HANDLER_HPP_

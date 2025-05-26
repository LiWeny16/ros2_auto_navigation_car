#include "auto_integration_test/error_handler.hpp"
#include <fstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <execinfo.h>
#include <cxxabi.h>

namespace auto_integration_test {

void ErrorHandler::registerErrorCallback(ErrorSeverity severity, ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    error_callbacks_[severity].push_back(callback);
}

void ErrorHandler::logError(ErrorSeverity severity, ErrorSource source, 
                           const std::string& message, const std::string& recovery_action) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 创建错误记录
    ErrorRecord record;
    record.severity = severity;
    record.source = source;
    record.message = message;
    record.timestamp = getCurrentTimestamp();
    record.stacktrace = (severity >= ErrorSeverity::ERROR) ? getStackTrace() : "";
    record.recovery_action = recovery_action;
    
    // 添加到历史记录
    error_history_.push_back(record);
    
    // 调用相应的错误处理回调
    auto it = error_callbacks_.find(severity);
    if (it != error_callbacks_.end()) {
        for (const auto& callback : it->second) {
            callback(record);
        }
    }
    
    // 根据严重性级别输出日志
    std::string source_str;
    switch (source) {
        case ErrorSource::PERCEPTION: source_str = "感知"; break;
        case ErrorSource::PLANNING: source_str = "规划"; break;
        case ErrorSource::CONTROL: source_str = "控制"; break;
        case ErrorSource::SIMULATION: source_str = "仿真"; break;
        case ErrorSource::SYSTEM: source_str = "系统"; break;
    }
    
    auto node = rclcpp::Node::make_shared("error_handler_temp");
    switch (severity) {
        case ErrorSeverity::INFO:
            RCLCPP_INFO(node->get_logger(), "[%s] %s", source_str.c_str(), message.c_str());
            break;
        case ErrorSeverity::WARNING:
            RCLCPP_WARN(node->get_logger(), "[%s] %s (恢复: %s)", 
                       source_str.c_str(), message.c_str(), recovery_action.c_str());
            break;
        case ErrorSeverity::ERROR:
            RCLCPP_ERROR(node->get_logger(), "[%s] %s (恢复: %s)", 
                       source_str.c_str(), message.c_str(), recovery_action.c_str());
            break;
        case ErrorSeverity::CRITICAL:
            RCLCPP_FATAL(node->get_logger(), "[%s] %s (恢复: %s)", 
                       source_str.c_str(), message.c_str(), recovery_action.c_str());
            break;
    }
}

std::vector<ErrorRecord> ErrorHandler::getErrorHistory() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_history_;
}

std::map<ErrorSeverity, int> ErrorHandler::getErrorStats(ErrorSource source) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::map<ErrorSeverity, int> stats;
    stats[ErrorSeverity::INFO] = 0;
    stats[ErrorSeverity::WARNING] = 0;
    stats[ErrorSeverity::ERROR] = 0;
    stats[ErrorSeverity::CRITICAL] = 0;
    
    for (const auto& record : error_history_) {
        if (record.source == source) {
            stats[record.severity]++;
        }
    }
    
    return stats;
}

std::string ErrorHandler::analyzeSystemHealth() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 计算各模块和严重性级别的错误数量
    std::map<ErrorSource, std::map<ErrorSeverity, int>> module_stats;
    
    for (const auto& record : error_history_) {
        module_stats[record.source][record.severity]++;
    }
    
    // 分析系统健康状况
    std::stringstream ss;
    ss << "系统健康状况分析:\n";
    
    bool has_critical = false;
    bool has_error = false;
    
    for (const auto& [source, stats] : module_stats) {
        std::string source_str;
        switch (source) {
            case ErrorSource::PERCEPTION: source_str = "感知模块"; break;
            case ErrorSource::PLANNING: source_str = "规划模块"; break;
            case ErrorSource::CONTROL: source_str = "控制模块"; break;
            case ErrorSource::SIMULATION: source_str = "仿真模块"; break;
            case ErrorSource::SYSTEM: source_str = "系统级"; break;
        }
        
        ss << "- " << source_str << ":\n";
        ss << "  信息: " << stats.at(ErrorSeverity::INFO) << ", ";
        ss << "警告: " << stats.at(ErrorSeverity::WARNING) << ", ";
        ss << "错误: " << stats.at(ErrorSeverity::ERROR) << ", ";
        ss << "严重错误: " << stats.at(ErrorSeverity::CRITICAL) << "\n";
        
        if (stats.at(ErrorSeverity::CRITICAL) > 0) {
            has_critical = true;
        }
        
        if (stats.at(ErrorSeverity::ERROR) > 0) {
            has_error = true;
        }
    }
    
    // 总体健康评估
    ss << "\n总体健康评估: ";
    if (has_critical) {
        ss << "严重 - 系统存在严重错误，需要立即修复";
    } else if (has_error) {
        ss << "警告 - 系统存在错误，功能可能受到影响";
    } else {
        ss << "良好 - 系统运行正常";
    }
    
    return ss.str();
}

void ErrorHandler::generateErrorReport(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        auto node = rclcpp::Node::make_shared("error_handler_temp");
        RCLCPP_ERROR(node->get_logger(), "无法打开文件保存错误报告: %s", filename.c_str());
        return;
    }
    
    // 写入报告标题
    file << "=================================\n";
    file << "自动驾驶仿真系统错误诊断报告\n";
    file << "生成时间: " << getCurrentTimestamp() << "\n";
    file << "=================================\n\n";
    
    // 写入系统健康分析
    file << analyzeSystemHealth() << "\n\n";
    
    // 写入详细错误记录
    file << "详细错误记录:\n";
    file << "---------------------------------\n";
    
    for (const auto& record : error_history_) {
        // 获取错误严重性和来源的字符串表示
        std::string severity_str;
        switch (record.severity) {
            case ErrorSeverity::INFO: severity_str = "信息"; break;
            case ErrorSeverity::WARNING: severity_str = "警告"; break;
            case ErrorSeverity::ERROR: severity_str = "错误"; break;
            case ErrorSeverity::CRITICAL: severity_str = "严重错误"; break;
        }
        
        std::string source_str;
        switch (record.source) {
            case ErrorSource::PERCEPTION: source_str = "感知模块"; break;
            case ErrorSource::PLANNING: source_str = "规划模块"; break;
            case ErrorSource::CONTROL: source_str = "控制模块"; break;
            case ErrorSource::SIMULATION: source_str = "仿真模块"; break;
            case ErrorSource::SYSTEM: source_str = "系统级"; break;
        }
        
        file << "时间: " << record.timestamp << "\n";
        file << "级别: " << severity_str << "\n";
        file << "来源: " << source_str << "\n";
        file << "消息: " << record.message << "\n";
        
        if (!record.recovery_action.empty()) {
            file << "恢复操作: " << record.recovery_action << "\n";
        }
        
        if (!record.stacktrace.empty()) {
            file << "堆栈跟踪:\n" << record.stacktrace << "\n";
        }
        
        file << "---------------------------------\n";
    }
    
    // 写入建议
    file << "\n改进建议:\n";
    
    // 根据错误情况提供建议
    std::map<ErrorSource, int> error_counts;
    for (const auto& record : error_history_) {
        if (record.severity >= ErrorSeverity::ERROR) {
            error_counts[record.source]++;
        }
    }
    
    for (const auto& [source, count] : error_counts) {
        if (count == 0) continue;
        
        std::string source_str;
        switch (source) {
            case ErrorSource::PERCEPTION:
                source_str = "感知模块";
                file << "- 感知模块存在 " << count << " 个错误，建议检查传感器数据处理和目标检测算法\n";
                break;
            case ErrorSource::PLANNING:
                source_str = "规划模块";
                file << "- 规划模块存在 " << count << " 个错误，建议优化路径规划算法，检查地图数据\n";
                break;
            case ErrorSource::CONTROL:
                source_str = "控制模块";
                file << "- 控制模块存在 " << count << " 个错误，建议调整控制参数，检查执行器模型\n";
                break;
            case ErrorSource::SIMULATION:
                source_str = "仿真模块";
                file << "- 仿真模块存在 " << count << " 个错误，建议检查仿真环境设置和物理参数\n";
                break;
            case ErrorSource::SYSTEM:
                source_str = "系统级";
                file << "- 系统级存在 " << count << " 个错误，建议检查系统集成和模块间通信\n";
                break;
        }
    }
    
    file.close();
    
    auto node = rclcpp::Node::make_shared("error_handler_temp");
    RCLCPP_INFO(node->get_logger(), "错误报告已保存到: %s", filename.c_str());
}

std::string ErrorHandler::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

std::string ErrorHandler::getStackTrace() const {
    // 获取当前堆栈
    const int max_frames = 10;
    void* callstack[max_frames];
    int frames = backtrace(callstack, max_frames);
    char** symbols = backtrace_symbols(callstack, frames);
    
    std::stringstream ss;
    for (int i = 0; i < frames; ++i) {
        ss << symbols[i] << "\n";
    }
    
    free(symbols);
    return ss.str();
}

} // namespace auto_integration_test

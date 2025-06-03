#ifndef AUTO_SIMULATION_JSON_UTILS_HPP
#define AUTO_SIMULATION_JSON_UTILS_HPP

#include <string>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/planning_request.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace auto_simulation {
namespace json_utils {

using json = nlohmann::json;

/**
 * @brief 消息转换为JSON字符串
 * 
 * @param msg 要转换的消息
 * @return std::string JSON字符串
 */
template<typename T>
std::string msg_to_json_string(const T& msg) {
    json j = msg;  // 依赖ADL方式的to_json
    return j.dump();
}

/**
 * @brief 从JSON字符串转换为消息
 * 
 * @param json_str JSON字符串
 * @return T 转换后的消息
 */
template<typename T>
T json_string_to_msg(const std::string& json_str) {
    json j = json::parse(json_str);
    return j.get<T>();  // 依赖ADL方式的from_json
}

/**
 * @brief 将消息封装到JSON标准消息中
 * 
 * @param msg 要封装的消息
 * @param topic_type 消息类型名称（可选，用于调试）
 * @return std_msgs::msg::String 封装后的JSON消息
 */
template<typename T>
std_msgs::msg::String wrap_as_json_msg(const T& msg, const std::string& topic_type = "") {
    std_msgs::msg::String json_msg;
    json j;
    j["data"] = msg;  // 实际消息数据
    if (!topic_type.empty()) {
        j["type"] = topic_type;  // 可选的类型信息
    }
    json_msg.data = j.dump();
    return json_msg;
}

/**
 * @brief 从JSON标准消息中提取特定类型的消息
 * 
 * @param json_msg 包含JSON的标准消息
 * @return T 提取的特定类型消息
 */
template<typename T>
T unwrap_from_json_msg(const std_msgs::msg::String& json_msg) {
    json j = json::parse(json_msg.data);
    return j["data"].get<T>();
}

/**
 * @brief 创建JSON消息发布者
 * 
 * @param node ROS2节点
 * @param topic 主题名称
 * @param qos QoS设置
 * @return 标准字符串消息发布者
 */
inline auto create_json_publisher(
    rclcpp::Node* node, 
    const std::string& topic, 
    size_t qos = 10) {
    return node->create_publisher<std_msgs::msg::String>(topic + "_json", qos);
}

/**
 * @brief 以JSON格式发布消息
 * 
 * @param publisher 发布者
 * @param msg 要发布的消息
 * @param topic_type 可选的类型名称
 */
template<typename T>
void publish_as_json(
    const typename rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher,
    const T& msg,
    const std::string& topic_type = "") {
    publisher->publish(wrap_as_json_msg(msg, topic_type));
}

/**
 * @brief 创建订阅JSON消息的订阅者
 * 
 * @param node ROS2节点
 * @param topic 主题名称
 * @param callback 回调函数，接收解析后的特定类型消息
 * @param qos QoS设置
 * @return 消息订阅者
 */
template<typename T, typename NodeT>
auto create_json_subscription(
    NodeT* node, 
    const std::string& topic, 
    std::function<void(const T&)> callback,
    size_t qos = 10) {
    
    auto wrapped_callback = [callback](const std_msgs::msg::String::SharedPtr json_msg) {
        callback(unwrap_from_json_msg<T>(*json_msg));
    };
    
    return node->template create_subscription<std_msgs::msg::String>(
        topic + "_json", qos, wrapped_callback);
}

} // namespace json_utils
} // namespace auto_simulation

// 添加各种消息类型的序列化/反序列化支持
namespace nlohmann {

// GridMap序列化和反序列化
template <>
struct adl_serializer<auto_msgs::msg::GridMap> {
    static void to_json(json& j, const auto_msgs::msg::GridMap& msg) {
        j = {
            {"header", {
                {"frame_id", msg.header.frame_id},
                {"stamp", {
                    {"sec", msg.header.stamp.sec},
                    {"nanosec", msg.header.stamp.nanosec}
                }}
            }},
            {"width", msg.width},
            {"height", msg.height},
            {"resolution", msg.resolution},
            {"origin", {
                {"position", {
                    {"x", msg.origin.position.x},
                    {"y", msg.origin.position.y},
                    {"z", msg.origin.position.z}
                }},
                {"orientation", {
                    {"x", msg.origin.orientation.x},
                    {"y", msg.origin.orientation.y},
                    {"z", msg.origin.orientation.z},
                    {"w", msg.origin.orientation.w}
                }}
            }},
            {"data", msg.data}
        };
    }
    
    static void from_json(const json& j, auto_msgs::msg::GridMap& msg) {
        msg.header.frame_id = j["header"]["frame_id"];
        msg.header.stamp.sec = j["header"]["stamp"]["sec"];
        msg.header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
        
        msg.width = j["width"];
        msg.height = j["height"];
        msg.resolution = j["resolution"];
        
        msg.origin.position.x = j["origin"]["position"]["x"];
        msg.origin.position.y = j["origin"]["position"]["y"];
        msg.origin.position.z = j["origin"]["position"]["z"];
        
        msg.origin.orientation.x = j["origin"]["orientation"]["x"];
        msg.origin.orientation.y = j["origin"]["orientation"]["y"];
        msg.origin.orientation.z = j["origin"]["orientation"]["z"];
        msg.origin.orientation.w = j["origin"]["orientation"]["w"];
        
        // Convert the JSON array to the correct type of msg.data
        auto data_array = j["data"].get<std::vector<int8_t>>();
        msg.data.assign(data_array.begin(), data_array.end());
    }
};

// PoseStamped序列化和反序列化
template <>
struct adl_serializer<geometry_msgs::msg::PoseStamped> {
    static void to_json(json& j, const geometry_msgs::msg::PoseStamped& msg) {
        j = {
            {"header", {
                {"frame_id", msg.header.frame_id},
                {"stamp", {
                    {"sec", msg.header.stamp.sec},
                    {"nanosec", msg.header.stamp.nanosec}
                }}
            }},
            {"pose", {
                {"position", {
                    {"x", msg.pose.position.x},
                    {"y", msg.pose.position.y},
                    {"z", msg.pose.position.z}
                }},
                {"orientation", {
                    {"x", msg.pose.orientation.x},
                    {"y", msg.pose.orientation.y},
                    {"z", msg.pose.orientation.z},
                    {"w", msg.pose.orientation.w}
                }}
            }}
        };
    }
    
    static void from_json(const json& j, geometry_msgs::msg::PoseStamped& msg) {
        msg.header.frame_id = j["header"]["frame_id"];
        msg.header.stamp.sec = j["header"]["stamp"]["sec"];
        msg.header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
        
        msg.pose.position.x = j["pose"]["position"]["x"];
        msg.pose.position.y = j["pose"]["position"]["y"];
        msg.pose.position.z = j["pose"]["position"]["z"];
        
        msg.pose.orientation.x = j["pose"]["orientation"]["x"];
        msg.pose.orientation.y = j["pose"]["orientation"]["y"];
        msg.pose.orientation.z = j["pose"]["orientation"]["z"];
        msg.pose.orientation.w = j["pose"]["orientation"]["w"];
    }
};

// PlanningRequest序列化和反序列化
template <>
struct adl_serializer<auto_msgs::msg::PlanningRequest> {
    static void to_json(json& j, const auto_msgs::msg::PlanningRequest& msg) {
        // 复用已有的PoseStamped序列化器处理start和goal
        json start_json, goal_json;
        adl_serializer<geometry_msgs::msg::PoseStamped>::to_json(start_json, msg.start);
        adl_serializer<geometry_msgs::msg::PoseStamped>::to_json(goal_json, msg.goal);
        
        j = {
            {"header", {
                {"frame_id", msg.header.frame_id},
                {"stamp", {
                    {"sec", msg.header.stamp.sec},
                    {"nanosec", msg.header.stamp.nanosec}
                }}
            }},
            {"start", start_json},
            {"goal", goal_json},
            {"planner_type", msg.planner_type},
            {"consider_kinematic", msg.consider_kinematic}
        };
    }
    
    static void from_json(const json& j, auto_msgs::msg::PlanningRequest& msg) {
        // 解析header
        if (j.contains("header")) {
            if (j["header"].contains("frame_id"))
                msg.header.frame_id = j["header"]["frame_id"].get<std::string>();
            if (j["header"].contains("stamp")) {
                if (j["header"]["stamp"].contains("sec"))
                    msg.header.stamp.sec = j["header"]["stamp"]["sec"].get<int32_t>();
                if (j["header"]["stamp"].contains("nanosec"))
                    msg.header.stamp.nanosec = j["header"]["stamp"]["nanosec"].get<uint32_t>();
            }
        }
        
        // 解析起点和终点，复用PoseStamped的反序列化
        if (j.contains("start"))
            adl_serializer<geometry_msgs::msg::PoseStamped>::from_json(j["start"], msg.start);
            
        if (j.contains("goal"))
            adl_serializer<geometry_msgs::msg::PoseStamped>::from_json(j["goal"], msg.goal);
        
        // 解析其他字段
        if (j.contains("planner_type"))
            msg.planner_type = j["planner_type"].get<std::string>();
        if (j.contains("consider_kinematic"))
            msg.consider_kinematic = j["consider_kinematic"].get<bool>();
    }
};

} // namespace nlohmann

#endif // AUTO_SIMULATION_JSON_UTILS_HPP
#include "auto_simulation/mqtt_bridge.hpp"
#include <cstring>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;
namespace auto_simulation {

MqttBridge::MqttBridge(const std::string& client_id, const std::string& host, int port)
    : client_id_(client_id), host_(host), port_(port), connected_(false) {
    
    // 初始化MQTT库
    mosquitto_lib_init();
    
    // 创建MQTT客户端实例
    mosq_ = mosquitto_new(client_id_.c_str(), true, this);
    if (!mosq_) {
        throw std::runtime_error("无法创建MQTT客户端实例");
    }
    
    // 设置回调函数
    mosquitto_connect_callback_set(mosq_, onConnect);
    mosquitto_disconnect_callback_set(mosq_, onDisconnect);
    mosquitto_message_callback_set(mosq_, onMessage);
}

MqttBridge::~MqttBridge() {
    if (connected_) {
        disconnect();
    }
    
    if (mosq_) {
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    
    mosquitto_lib_cleanup();
}

bool MqttBridge::connect() {
    int rc = mosquitto_connect(mosq_, host_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "无法连接到MQTT代理 %s:%d: %s", 
            host_.c_str(), port_, mosquitto_strerror(rc));
        return false;
    }
    
    // 启动网络循环
    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "无法启动MQTT网络循环: %s", mosquitto_strerror(rc));
        return false;
    }
    
    return true;
}

void MqttBridge::disconnect() {
    if (mosq_ && connected_) {
        mosquitto_disconnect(mosq_);
        mosquitto_loop_stop(mosq_, true);
        connected_ = false;
    }
}

bool MqttBridge::publish(const std::string& topic, const std::string& payload, int qos) {
    if (!mosq_ || !connected_) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), "MQTT客户端未连接");
        return false;
    }
    
    int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), 
        static_cast<int>(payload.size()), payload.c_str(), qos, false);
        
    if (rc != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "发布消息失败: %s", mosquitto_strerror(rc));
        return false;
    }
    
    return true;
}

bool MqttBridge::subscribe(const std::string& topic, int qos) {
    if (!mosq_ || !connected_) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), "MQTT客户端未连接");
        return false;
    }
    
    int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    if (rc != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "订阅主题失败: %s", mosquitto_strerror(rc));
        return false;
    }
    
    return true;
}

bool MqttBridge::unsubscribe(const std::string& topic) {
    if (!mosq_ || !connected_) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), "MQTT客户端未连接");
        return false;
    }
    
    int rc = mosquitto_unsubscribe(mosq_, nullptr, topic.c_str());
    if (rc != MOSQ_ERR_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "取消订阅主题失败: %s", mosquitto_strerror(rc));
        return false;
    }
    
    return true;
}

void MqttBridge::setMessageCallback(MqttMessageCallback callback) {
    message_callback_ = callback;
}

void MqttBridge::spin(int timeout_ms) {
    if (mosq_ && connected_) {
        mosquitto_loop(mosq_, timeout_ms, 1);
    }
}

bool MqttBridge::publishGridMap(const auto_msgs::msg::GridMap& map) {
    try {
        json j;
        j["header"]["frame_id"] = map.header.frame_id;
        j["header"]["stamp"]["sec"] = map.header.stamp.sec;
        j["header"]["stamp"]["nanosec"] = map.header.stamp.nanosec;
        j["width"] = map.width;
        j["height"] = map.height;
        j["resolution"] = map.resolution;
        j["origin"]["position"]["x"] = map.origin.position.x;
        j["origin"]["position"]["y"] = map.origin.position.y;
        j["origin"]["position"]["z"] = map.origin.position.z;
        j["origin"]["orientation"]["x"] = map.origin.orientation.x;
        j["origin"]["orientation"]["y"] = map.origin.orientation.y;
        j["origin"]["orientation"]["z"] = map.origin.orientation.z;
        j["origin"]["orientation"]["w"] = map.origin.orientation.w;
        
        // 将地图数据转换为base64编码以减小大小
        std::vector<uint8_t> data;
        for (auto value : map.data) {
            data.push_back(static_cast<uint8_t>(value));
        }
        j["data"] = data;
        
        return publish("auto_driving/grid_map", j.dump());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "发布网格地图失败: %s", e.what());
        return false;
    }
}

bool MqttBridge::publishVehiclePose(const geometry_msgs::msg::PoseStamped& pose) {
    try {
        json j;
        j["header"]["frame_id"] = pose.header.frame_id;
        j["header"]["stamp"]["sec"] = pose.header.stamp.sec;
        j["header"]["stamp"]["nanosec"] = pose.header.stamp.nanosec;
        j["pose"]["position"]["x"] = pose.pose.position.x;
        j["pose"]["position"]["y"] = pose.pose.position.y;
        j["pose"]["position"]["z"] = pose.pose.position.z;
        j["pose"]["orientation"]["x"] = pose.pose.orientation.x;
        j["pose"]["orientation"]["y"] = pose.pose.orientation.y;
        j["pose"]["orientation"]["z"] = pose.pose.orientation.z;
        j["pose"]["orientation"]["w"] = pose.pose.orientation.w;
        
        return publish("auto_driving/vehicle_pose", j.dump());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "发布车辆位姿失败: %s", e.what());
        return false;
    }
}

bool MqttBridge::publishPlanningPath(const auto_msgs::msg::PlanningPath& path) {
    try {
        json j;
        j["header"]["frame_id"] = path.header.frame_id;
        j["header"]["stamp"]["sec"] = path.header.stamp.sec;
        j["header"]["stamp"]["nanosec"] = path.header.stamp.nanosec;
        j["planner_type"] = path.planner_type;
        j["total_distance"] = path.total_distance;
        j["planning_time"] = path.planning_time;
        
        json points = json::array();
        for (const auto& point : path.points) {
            json p;
            p["pose"]["position"]["x"] = point.pose.position.x;
            p["pose"]["position"]["y"] = point.pose.position.y;
            p["pose"]["position"]["z"] = point.pose.position.z;
            p["pose"]["orientation"]["x"] = point.pose.orientation.x;
            p["pose"]["orientation"]["y"] = point.pose.orientation.y;
            p["pose"]["orientation"]["z"] = point.pose.orientation.z;
            p["pose"]["orientation"]["w"] = point.pose.orientation.w;
            p["steering_angle"] = point.steering_angle;
            p["velocity"] = point.velocity;
            p["curvature"] = point.curvature;
            points.push_back(p);
        }
        j["points"] = points;
        
        return publish("auto_driving/planning_path", j.dump());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "发布规划路径失败: %s", e.what());
        return false;
    }
}

auto_msgs::msg::GridMap MqttBridge::parseGridMap(const std::string& json_str) {
    auto_msgs::msg::GridMap map;
    
    try {
        json j = json::parse(json_str);
        
        map.header.frame_id = j["header"]["frame_id"];
        map.header.stamp.sec = j["header"]["stamp"]["sec"];
        map.header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
        map.width = j["width"];
        map.height = j["height"];
        map.resolution = j["resolution"];
        map.origin.position.x = j["origin"]["position"]["x"];
        map.origin.position.y = j["origin"]["position"]["y"];
        map.origin.position.z = j["origin"]["position"]["z"];
        map.origin.orientation.x = j["origin"]["orientation"]["x"];
        map.origin.orientation.y = j["origin"]["orientation"]["y"];
        map.origin.orientation.z = j["origin"]["orientation"]["z"];
        map.origin.orientation.w = j["origin"]["orientation"]["w"];
        
        // 解析地图数据
        auto data = j["data"].get<std::vector<uint8_t>>();
        map.data.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            map.data[i] = static_cast<int8_t>(data[i]);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "解析网格地图失败: %s", e.what());
    }
    
    return map;
}

geometry_msgs::msg::PoseStamped MqttBridge::parsePoseStamped(const std::string& json_str) {
    geometry_msgs::msg::PoseStamped pose;
    
    try {
        json j = json::parse(json_str);
        
        pose.header.frame_id = j["header"]["frame_id"];
        pose.header.stamp.sec = j["header"]["stamp"]["sec"];
        pose.header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
        pose.pose.position.x = j["pose"]["position"]["x"];
        pose.pose.position.y = j["pose"]["position"]["y"];
        pose.pose.position.z = j["pose"]["position"]["z"];
        pose.pose.orientation.x = j["pose"]["orientation"]["x"];
        pose.pose.orientation.y = j["pose"]["orientation"]["y"];
        pose.pose.orientation.z = j["pose"]["orientation"]["z"];
        pose.pose.orientation.w = j["pose"]["orientation"]["w"];
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "解析位姿失败: %s", e.what());
    }
    
    return pose;
}

// 静态回调函数
void MqttBridge::onConnect(struct mosquitto* mosq, void* obj, int rc) {
    auto* bridge = static_cast<MqttBridge*>(obj);
    if (rc == 0) {
        bridge->connected_ = true;
        RCLCPP_INFO(rclcpp::get_logger("mqtt_bridge"), 
            "已连接到MQTT代理 %s:%d", bridge->host_.c_str(), bridge->port_);
    } else {
        bridge->connected_ = false;
        RCLCPP_ERROR(rclcpp::get_logger("mqtt_bridge"), 
            "连接到MQTT代理失败: %s", mosquitto_connack_string(rc));
    }
}

void MqttBridge::onDisconnect(struct mosquitto* mosq, void* obj, int reason) {
    auto* bridge = static_cast<MqttBridge*>(obj);
    bridge->connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("mqtt_bridge"), 
        "与MQTT代理断开连接: %s", mosquitto_strerror(reason));
}

void MqttBridge::onMessage(struct mosquitto* mosq, void* obj, 
                         const struct mosquitto_message* message) {
    auto* bridge = static_cast<MqttBridge*>(obj);
    
    if (bridge->message_callback_) {
        std::string topic = message->topic;
        std::string payload;
        
        if (message->payload) {
            payload = std::string(static_cast<const char*>(message->payload), message->payloadlen);
        }
        
        bridge->message_callback_(topic, payload);
    }
}

} // namespace auto_simulation

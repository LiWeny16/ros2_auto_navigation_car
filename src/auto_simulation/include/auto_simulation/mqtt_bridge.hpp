#ifndef MQTT_BRIDGE_HPP_
#define MQTT_BRIDGE_HPP_

#include <string>
#include <memory>
#include <functional>
#include <mosquitto.h>
#include "auto_msgs/msg/planning_path.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace auto_simulation {

// MQTT消息回调函数的类型定义
using MqttMessageCallback = std::function<void(const std::string& topic, const std::string& payload)>;

class MqttBridge {
public:
    MqttBridge(const std::string& client_id, const std::string& host = "localhost", int port = 1883);
    virtual ~MqttBridge();

    // 连接到MQTT代理
    bool connect();
    
    // 断开与MQTT代理的连接
    void disconnect();
    
    // 发布消息
    bool publish(const std::string& topic, const std::string& payload, int qos = 0);
    
    // 订阅主题
    bool subscribe(const std::string& topic, int qos = 0);
    
    // 取消订阅主题
    bool unsubscribe(const std::string& topic);
    
    // 设置消息回调
    void setMessageCallback(MqttMessageCallback callback);
    
    // 处理MQTT事件
    void spin(int timeout_ms = 100);
    
    // 发布自动驾驶系统相关消息
    bool publishGridMap(const auto_msgs::msg::GridMap& map);
    bool publishVehiclePose(const geometry_msgs::msg::PoseStamped& pose);
    bool publishPlanningPath(const auto_msgs::msg::PlanningPath& path);
    
    // 新增：自动泊车相关消息发布
    bool publishVehicleState(const geometry_msgs::msg::PoseStamped& pose, 
                           const geometry_msgs::msg::Twist& velocity,
                           const std::string& control_mode = "auto");
    bool publishParkingSpots(const std::vector<std::string>& parking_spots_json);
    bool publishManualControl(double steering, double throttle, double brake, bool reverse);
    bool publishParkingStatus(const std::string& status, const std::string& target_spot_id = "");
    bool publishEnvironmentObjects(const std::string& objects_json);
    
    // 从JSON解析消息
    auto_msgs::msg::GridMap parseGridMap(const std::string& json);
    geometry_msgs::msg::PoseStamped parsePoseStamped(const std::string& json);
    
    // 新增：解析泊车相关消息
    geometry_msgs::msg::Twist parseManualControl(const std::string& json);
    std::string parseParkingCommand(const std::string& json);

private:
    // MQTT回调函数
    static void onConnect(struct mosquitto* mosq, void* obj, int rc);
    static void onDisconnect(struct mosquitto* mosq, void* obj, int reason);
    static void onMessage(struct mosquitto* mosq, void* obj, const struct mosquitto_message* message);
    
    // MQTT客户端
    struct mosquitto* mosq_;
    
    // MQTT代理信息
    std::string client_id_;
    std::string host_;
    int port_;
    
    // 消息回调
    MqttMessageCallback message_callback_;
    
    // 连接状态
    bool connected_;
};

} // namespace auto_simulation

#endif // MQTT_BRIDGE_HPP_

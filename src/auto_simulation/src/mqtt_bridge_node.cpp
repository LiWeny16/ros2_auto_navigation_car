#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "auto_msgs/msg/grid_map.hpp"
#include "auto_msgs/msg/planning_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auto_simulation/mqtt_bridge.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace auto_simulation {

class MqttBridgeNode : public rclcpp::Node {
public:
    MqttBridgeNode() : Node("mqtt_bridge_node") {
        // 从参数获取MQTT设置
        this->declare_parameter("mqtt_host", "localhost");
        this->declare_parameter("mqtt_port", 1883);
        this->declare_parameter("mqtt_client_id", "auto_driving_bridge");
        
        std::string mqtt_host = this->get_parameter("mqtt_host").as_string();
        int mqtt_port = this->get_parameter("mqtt_port").as_int();
        std::string mqtt_client_id = this->get_parameter("mqtt_client_id").as_string();
        
        // 创建MQTT桥接
        try {
            mqtt_bridge_ = std::make_unique<MqttBridge>(
                mqtt_client_id, mqtt_host, mqtt_port);
                
            mqtt_bridge_->setMessageCallback(
                std::bind(&MqttBridgeNode::mqttMessageCallback, this, 
                          std::placeholders::_1, std::placeholders::_2));
                          
            if (!mqtt_bridge_->connect()) {
                RCLCPP_ERROR(this->get_logger(), "无法连接到MQTT代理");
            } else {
                // 订阅相关主题
                mqtt_bridge_->subscribe("auto_driving/command/#");
                RCLCPP_INFO(this->get_logger(), "已连接到MQTT代理");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "创建MQTT桥接失败: %s", e.what());
        }
        
        // 创建ROS订阅者
        map_sub_ = this->create_subscription<auto_msgs::msg::GridMap>(
            "grid_map", 10, std::bind(&MqttBridgeNode::mapCallback, this, _1));
            
        path_sub_ = this->create_subscription<auto_msgs::msg::PlanningPath>(
            "planning_path", 10, std::bind(&MqttBridgeNode::pathCallback, this, _1));
            
        // 创建定时器
        mqtt_timer_ = this->create_wall_timer(
            10ms, std::bind(&MqttBridgeNode::processMessages, this));
            
        RCLCPP_INFO(this->get_logger(), "MQTT桥接节点已启动");
    }
    
    ~MqttBridgeNode() {
        if (mqtt_bridge_) {
            mqtt_bridge_->disconnect();
        }
    }

private:
    void mapCallback(const auto_msgs::msg::GridMap::SharedPtr msg) {
        if (mqtt_bridge_ && msg) {
            mqtt_bridge_->publishGridMap(*msg);
        }
    }
    
    void pathCallback(const auto_msgs::msg::PlanningPath::SharedPtr msg) {
        if (mqtt_bridge_ && msg) {
            mqtt_bridge_->publishPlanningPath(*msg);
        }
    }
    
    void mqttMessageCallback(const std::string& topic, const std::string& payload) {
        RCLCPP_INFO(this->get_logger(), "收到MQTT消息: %s", topic.c_str());
        
        try {
            if (topic == "auto_driving/command/request_map") {
                // TODO: 请求最新的地图
            } else if (topic == "auto_driving/command/set_goal") {
                // TODO: 设置新的目标点
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理MQTT消息失败: %s", e.what());
        }
    }
    
    void processMessages() {
        if (mqtt_bridge_) {
            mqtt_bridge_->spin(0);
        }
    }
    
    std::unique_ptr<MqttBridge> mqtt_bridge_;
    
    rclcpp::Subscription<auto_msgs::msg::GridMap>::SharedPtr map_sub_;
    rclcpp::Subscription<auto_msgs::msg::PlanningPath>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr mqtt_timer_;
};

} // namespace auto_simulation

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<auto_simulation::MqttBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#pragma once

#include "../entities/vehicle_state.hpp"
#include "../entities/parking_spot.hpp"
#include "auto_simulation/mqtt_bridge.hpp"
#include <memory>
#include <functional>
#include <nlohmann/json.hpp>

namespace auto_parking::interface_adapters {

/**
 * @brief 自动泊车MQTT适配器 - Clean Architecture Interface Adapters层
 * 
 * 使用现有的auto_simulation::MqttBridge，为自动泊车系统提供MQTT通信
 * 遵循Clean Architecture原则，适配外部MQTT框架
 * 
 * 参考: Clean Architecture - https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html
 */
class ParkingMqttAdapter {
public:
    // 回调函数类型定义
    using ManualControlCallback = std::function<void(const entities::VehicleState::Control&)>;
    using ParkingCommandCallback = std::function<void(const std::string&, const std::string&)>;
    
    explicit ParkingMqttAdapter(const std::string& client_id = "parking_simulator");
    ~ParkingMqttAdapter() = default;
    
    // 连接管理
    bool connect();
    void disconnect();
    bool is_connected() const;
    
    // 发布车辆状态到前端
    bool publish_vehicle_state(const entities::VehicleState& state);
    
    // 发布泊车位信息
    bool publish_parking_spots(const std::vector<entities::ParkingSpot>& spots);
    
    // 发布泊车状态
    bool publish_parking_status(const std::string& status, 
                              const std::string& target_spot_id = "",
                              double progress = 0.0);
    
    // 发布环境对象（树木、障碍物等）
    bool publish_environment_objects(const std::vector<EnvironmentObject>& objects);
    
    // 发布3D可视化数据
    bool publish_3d_scene_update(const std::string& scene_data);
    
    // 订阅前端控制命令
    bool subscribe_manual_control(ManualControlCallback callback);
    bool subscribe_parking_commands(ParkingCommandCallback callback);
    
    // 处理MQTT消息
    void spin(int timeout_ms = 100);
    
    // MQTT主题常量
    struct Topics {
        static constexpr const char* VEHICLE_STATE = "parking_sim/vehicle/state";
        static constexpr const char* VEHICLE_CONTROL = "parking_sim/vehicle/control";
        static constexpr const char* PARKING_SPOTS = "parking_sim/environment/parking_spots";
        static constexpr const char* PARKING_STATUS = "parking_sim/status/parking";
        static constexpr const char* ENVIRONMENT_OBJECTS = "parking_sim/environment/objects";
        static constexpr const char* SCENE_3D = "parking_sim/visualization/3d";
        static constexpr const char* MANUAL_CONTROL = "parking_sim/control/manual";
        static constexpr const char* PARKING_COMMANDS = "parking_sim/commands/parking";
    };
    
private:
    // 环境对象结构
    struct EnvironmentObject {
        std::string id;
        std::string type;  // "tree", "obstacle", "building", etc.
        double x, y, z;
        double rotation;
        double scale_x{1.0}, scale_y{1.0}, scale_z{1.0};
        std::string model_path;
    };
    
    std::unique_ptr<auto_simulation::MqttBridge> mqtt_bridge_;
    
    // 回调函数
    ManualControlCallback manual_control_callback_;
    ParkingCommandCallback parking_command_callback_;
    
    // 消息处理
    void handle_mqtt_message(const std::string& topic, const std::string& payload);
    
    // JSON转换辅助函数
    nlohmann::json vehicle_state_to_json(const entities::VehicleState& state);
    nlohmann::json parking_spot_to_json(const entities::ParkingSpot& spot);
    nlohmann::json environment_object_to_json(const EnvironmentObject& obj);
    
    entities::VehicleState::Control json_to_control(const nlohmann::json& j);
    
    // 验证JSON格式
    bool validate_control_json(const nlohmann::json& j);
    bool validate_command_json(const nlohmann::json& j);
};

} // namespace auto_parking::interface_adapters 
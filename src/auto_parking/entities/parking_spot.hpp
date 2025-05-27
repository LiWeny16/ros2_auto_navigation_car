#pragma once

#include <vector>
#include <string>

namespace auto_parking::entities {

/**
 * @brief 泊车位实体 - Clean Architecture Entities层
 * 
 * 定义泊车位的几何信息和状态
 * 遵循Clean Architecture原则，纯业务逻辑实体
 * 
 * 参考: Clean Architecture - https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html
 */
class ParkingSpot {
public:
    // 泊车位类型
    enum class Type {
        PARALLEL,       // 平行泊车
        PERPENDICULAR,  // 垂直泊车
        DIAGONAL        // 斜向泊车
    };
    
    // 泊车位状态
    enum class Status {
        AVAILABLE,      // 可用
        OCCUPIED,       // 被占用
        RESERVED,       // 预留
        BLOCKED         // 被阻挡
    };
    
    // 构造函数
    ParkingSpot() = default;
    
    ParkingSpot(const std::string& id, Type type, double x, double y, double theta, 
                double width, double length)
        : id_(id), type_(type), center_x_(x), center_y_(y), orientation_(theta),
          width_(width), length_(length), status_(Status::AVAILABLE) {}
    
    // 基本属性
    std::string id_;                    // 泊车位ID
    Type type_{Type::PERPENDICULAR};    // 泊车位类型
    Status status_{Status::AVAILABLE};  // 泊车位状态
    
    // 几何信息
    double center_x_{0.0};              // 中心点X坐标 (米)
    double center_y_{0.0};              // 中心点Y坐标 (米)
    double orientation_{0.0};           // 朝向角度 (弧度)
    double width_{2.5};                 // 宽度 (米)
    double length_{5.0};                // 长度 (米)
    
    // 边界点 (按逆时针顺序)
    struct Corner {
        double x, y;
        Corner(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
    };
    
    std::vector<Corner> corners_;       // 四个角点
    
    // 入口信息
    struct Entry {
        double x, y;                    // 入口点坐标
        double approach_angle;          // 接近角度
        double min_approach_distance;   // 最小接近距离
        Entry(double x_val = 0.0, double y_val = 0.0, double angle = 0.0, double dist = 3.0)
            : x(x_val), y(y_val), approach_angle(angle), min_approach_distance(dist) {}
    };
    
    Entry entry_point_;                 // 入口点
    
    // 难度评级
    enum class Difficulty {
        EASY = 1,       // 简单 - 宽敞空间
        MEDIUM = 2,     // 中等 - 标准空间
        HARD = 3,       // 困难 - 紧凑空间
        EXPERT = 4      // 专家 - 极限空间
    };
    
    Difficulty difficulty_{Difficulty::MEDIUM};
    
    // 业务方法
    bool is_available() const {
        return status_ == Status::AVAILABLE;
    }
    
    bool can_fit_vehicle(double vehicle_width, double vehicle_length) const {
        const double margin = 0.3; // 30cm安全边距
        return (width_ >= vehicle_width + margin) && (length_ >= vehicle_length + margin);
    }
    
    double get_area() const {
        return width_ * length_;
    }
    
    // 计算与车辆的距离
    double distance_to_vehicle(double vehicle_x, double vehicle_y) const {
        double dx = center_x_ - vehicle_x;
        double dy = center_y_ - vehicle_y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // 生成角点坐标
    void calculate_corners() {
        corners_.clear();
        
        double half_width = width_ / 2.0;
        double half_length = length_ / 2.0;
        double cos_theta = std::cos(orientation_);
        double sin_theta = std::sin(orientation_);
        
        // 四个角点 (逆时针)
        std::vector<std::pair<double, double>> local_corners = {
            {-half_length, -half_width},  // 左后
            {half_length, -half_width},   // 右后
            {half_length, half_width},    // 右前
            {-half_length, half_width}    // 左前
        };
        
        for (const auto& local : local_corners) {
            double global_x = center_x_ + local.first * cos_theta - local.second * sin_theta;
            double global_y = center_y_ + local.first * sin_theta + local.second * cos_theta;
            corners_.emplace_back(global_x, global_y);
        }
    }
    
    // 检查点是否在泊车位内
    bool contains_point(double x, double y) const {
        if (corners_.size() != 4) return false;
        
        // 使用射线法判断点是否在多边形内
        bool inside = false;
        for (size_t i = 0, j = corners_.size() - 1; i < corners_.size(); j = i++) {
            if (((corners_[i].y > y) != (corners_[j].y > y)) &&
                (x < (corners_[j].x - corners_[i].x) * (y - corners_[i].y) / 
                     (corners_[j].y - corners_[i].y) + corners_[i].x)) {
                inside = !inside;
            }
        }
        return inside;
    }
    
    // 设置入口点 (基于泊车位类型自动计算)
    void calculate_entry_point() {
        double entry_distance = 3.0; // 入口点距离泊车位的距离
        
        switch (type_) {
            case Type::PERPENDICULAR:
                // 垂直泊车：入口在泊车位前方
                entry_point_.x = center_x_ - entry_distance * std::cos(orientation_);
                entry_point_.y = center_y_ - entry_distance * std::sin(orientation_);
                entry_point_.approach_angle = orientation_;
                break;
                
            case Type::PARALLEL:
                // 平行泊车：入口在泊车位侧方
                entry_point_.x = center_x_ - entry_distance * std::sin(orientation_);
                entry_point_.y = center_y_ + entry_distance * std::cos(orientation_);
                entry_point_.approach_angle = orientation_ - M_PI / 2;
                break;
                
            case Type::DIAGONAL:
                // 斜向泊车：入口在斜前方
                double diagonal_angle = orientation_ - M_PI / 4;
                entry_point_.x = center_x_ - entry_distance * std::cos(diagonal_angle);
                entry_point_.y = center_y_ - entry_distance * std::sin(diagonal_angle);
                entry_point_.approach_angle = diagonal_angle;
                break;
        }
        
        entry_point_.min_approach_distance = entry_distance;
    }
    
    // 验证泊车位有效性
    bool is_valid() const {
        return !id_.empty() && 
               width_ > 0 && length_ > 0 &&
               std::isfinite(center_x_) && std::isfinite(center_y_) && 
               std::isfinite(orientation_);
    }
};

} // namespace auto_parking::entities 
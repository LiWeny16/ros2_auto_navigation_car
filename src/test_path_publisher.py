#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 路径发布者用于测试控制模块的运行情况

import rclpy
from rclpy.node import Node
from auto_msgs.msg import PlanningPath, PathPoint
from geometry_msgs.msg import Quaternion
import math
import time
import numpy as np
from tf_transformations import quaternion_from_euler

class TestPathPublisher(Node):
    def __init__(self):
        super().__init__('test_path_publisher')
        self.path_publisher = self.create_publisher(PlanningPath, 'planning_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.get_logger().info('测试路径发布者已启动')
        
    def publish_path(self):
        # 创建一个简单的路径：直线 + 圆弧 + 直线
        path_msg = PlanningPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        path_msg.planner_type = 'test'
        
        # 路径点数组
        points = []
        total_distance = 0.0
        last_point = None
        
        # 第一段：起点到直线终点
        for i in range(20):
            t = i / 19.0
            x = t * 10.0
            y = 0.0
            
            point = PathPoint()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.0
            
            q = quaternion_from_euler(0, 0, 0)
            point.pose.orientation.x = q[0]
            point.pose.orientation.y = q[1]
            point.pose.orientation.z = q[2]
            point.pose.orientation.w = q[3]
            
            point.velocity = 2.0
            point.steering_angle = 0.0
            point.curvature = 0.0
            
            points.append(point)
            
            # 计算累计距离
            if last_point is not None:
                dx = point.pose.position.x - last_point.pose.position.x
                dy = point.pose.position.y - last_point.pose.position.y
                segment_dist = math.sqrt(dx*dx + dy*dy)
                total_distance += segment_dist
            
            last_point = point
        
        # 第二段：90度右转弯 (圆弧)
        center_x = 10.0
        center_y = 10.0
        radius = 10.0
        start_angle = -math.pi/2
        end_angle = 0.0
        
        for i in range(20):
            t = i / 19.0
            angle = start_angle + t * (end_angle - start_angle)
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            point = PathPoint()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.0
            
            # 切线方向：垂直于半径方向
            yaw = angle + math.pi/2
            
            q = quaternion_from_euler(0, 0, yaw)
            point.pose.orientation.x = q[0]
            point.pose.orientation.y = q[1]
            point.pose.orientation.z = q[2]
            point.pose.orientation.w = q[3]
            
            point.velocity = 2.0
            point.steering_angle = 0.0
            point.curvature = 1.0 / radius
            
            points.append(point)
            
            # 计算累计距离
            if last_point is not None:
                dx = point.pose.position.x - last_point.pose.position.x
                dy = point.pose.position.y - last_point.pose.position.y
                segment_dist = math.sqrt(dx*dx + dy*dy)
                total_distance += segment_dist
            
            last_point = point
        
        # 第三段：最后一段直线
        for i in range(20):
            t = i / 19.0
            x = 20.0 + t * 10.0
            y = 10.0
            
            point = PathPoint()
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.0
            
            q = quaternion_from_euler(0, 0, 0)
            point.pose.orientation.x = q[0]
            point.pose.orientation.y = q[1]
            point.pose.orientation.z = q[2]
            point.pose.orientation.w = q[3]
            
            point.velocity = 2.0
            point.steering_angle = 0.0
            point.curvature = 0.0
            
            points.append(point)
            
            # 计算累计距离
            if last_point is not None:
                dx = point.pose.position.x - last_point.pose.position.x
                dy = point.pose.position.y - last_point.pose.position.y
                segment_dist = math.sqrt(dx*dx + dy*dy)
                total_distance += segment_dist
            
            last_point = point
        
        path_msg.points = points
        path_msg.total_distance = total_distance
        
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'已发布测试路径, 包含 {len(points)} 个点，总长度 {total_distance:.2f} 米')

def main(args=None):
    rclpy.init(args=args)
    node = TestPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

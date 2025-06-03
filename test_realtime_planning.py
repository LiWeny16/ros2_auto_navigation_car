#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from auto_msgs.msg import GridMap, PlanningPath, PlanningRequest
from std_msgs.msg import Empty
import time
from datetime import datetime

class RealtimePlanningTester(Node):
    def __init__(self):
        super().__init__('realtime_planning_tester')
        
        # 订阅地图和路径
        self.map_sub = self.create_subscription(
            GridMap, 'grid_map', self.map_callback, 10)
        self.path_sub = self.create_subscription(
            PlanningPath, 'planning_path', self.path_callback, 10)
        self.request_sub = self.create_subscription(
            PlanningRequest, 'planning_request', self.request_callback, 10)
        
        # 存储时间戳
        self.map_timestamps = []
        self.request_timestamps = []
        self.path_timestamps = []
        
        self.get_logger().info("实时规划测试器已启动")
        self.get_logger().info("监控地图发布 -> 规划请求 -> 路径生成的时间序列")
        
    def map_callback(self, msg):
        timestamp = time.time()
        self.map_timestamps.append(timestamp)
        
        self.get_logger().info(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] 📍 地图发布 (尺寸: {msg.width}x{msg.height})")
        
        # 清理旧数据，只保留最近10条记录
        if len(self.map_timestamps) > 10:
            self.map_timestamps.pop(0)
            
    def request_callback(self, msg):
        timestamp = time.time()
        self.request_timestamps.append(timestamp)
        
        # 计算从地图发布到规划请求的延迟
        if self.map_timestamps:
            delay_ms = (timestamp - self.map_timestamps[-1]) * 1000
            self.get_logger().info(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] 🎯 规划请求 (类型: {msg.planner_type}, 地图延迟: {delay_ms:.1f}ms)")
        else:
            self.get_logger().info(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] 🎯 规划请求 (类型: {msg.planner_type})")
            
        # 清理旧数据
        if len(self.request_timestamps) > 10:
            self.request_timestamps.pop(0)
            
    def path_callback(self, msg):
        timestamp = time.time()
        self.path_timestamps.append(timestamp)
        
        delays = []
        
        # 计算从地图发布到路径生成的总延迟
        if self.map_timestamps:
            total_delay_ms = (timestamp - self.map_timestamps[-1]) * 1000
            delays.append(f"总延迟: {total_delay_ms:.1f}ms")
            
        # 计算从规划请求到路径生成的延迟
        if self.request_timestamps:
            planning_delay_ms = (timestamp - self.request_timestamps[-1]) * 1000
            delays.append(f"规划延迟: {planning_delay_ms:.1f}ms")
            
        delay_info = ", ".join(delays) if delays else ""
        
        if msg.points:
            self.get_logger().info(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] ✅ 路径生成 (点数: {len(msg.points)}, 长度: {msg.total_distance:.1f}m, 耗时: {msg.planning_time:.3f}s, {delay_info})")
            
            # 评估响应性能
            if self.map_timestamps and self.request_timestamps:
                map_to_request = (self.request_timestamps[-1] - self.map_timestamps[-1]) * 1000
                request_to_path = (timestamp - self.request_timestamps[-1]) * 1000
                total_response = (timestamp - self.map_timestamps[-1]) * 1000
                
                # 性能评估
                if total_response < 500:
                    performance = "🟢 优秀"
                elif total_response < 1000:
                    performance = "🟡 良好"
                else:
                    performance = "🔴 需优化"
                    
                self.get_logger().info(f"    📊 性能分析: {performance} | 地图→请求: {map_to_request:.1f}ms | 请求→路径: {request_to_path:.1f}ms | 总响应: {total_response:.1f}ms")
        else:
            self.get_logger().warn(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] ❌ 路径生成失败 ({delay_info})")
            
        # 清理旧数据
        if len(self.path_timestamps) > 10:
            self.path_timestamps.pop(0)

def main(args=None):
    rclpy.init(args=args)
    
    tester = RealtimePlanningTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("测试器被中断")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
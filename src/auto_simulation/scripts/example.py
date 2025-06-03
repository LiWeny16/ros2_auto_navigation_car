#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from auto_msgs.msg import GridMap, PlanningPath, PlanningRequest

class SimpleBridge(Node):
    def __init__(self):
        super().__init__('simple_bridge')
        
        # 订阅C++节点发布的话题
        self.map_sub = self.create_subscription(
            GridMap,                    # 消息类型
            'grid_map',                 # 话题名
            self.map_callback,          # 回调函数
            10                          # 队列大小
        )
        
        self.path_sub = self.create_subscription(
            PlanningPath,
            'planning_path', 
            self.path_callback,
            10
        )
        
        # 发布给C++节点的话题
        self.request_pub = self.create_publisher(
            PlanningRequest,            # 消息类型
            'planning_request',         # 话题名
            10                          # 队列大小
        )
        
        self.get_logger().info("Python桥接节点启动")

    def map_callback(self, msg):
        """接收C++仿真节点发布的地图"""
        self.get_logger().info(f"收到地图: {msg.width}x{msg.height}")
        # 处理地图数据...
        
    def path_callback(self, msg):
        """接收C++规划节点发布的路径"""
        self.get_logger().info(f"收到路径: {len(msg.points)}个点")
        # 处理路径数据...
        
    def send_planning_request(self):
        """发送规划请求给C++节点"""
        msg = PlanningRequest()
        msg.planner_type = "astar"
        # 设置起点终点...
        
        self.request_pub.publish(msg)
        self.get_logger().info("发送规划请求")

def main():
    rclpy.init()
    node = SimpleBridge()
    
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
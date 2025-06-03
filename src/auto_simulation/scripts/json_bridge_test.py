#!/usr/bin/env python3
"""
JSON桥接测试节点
用于测试ROS2系统中的JSON格式消息转发
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import logging

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class JSONBridgeTest(Node):
    def __init__(self):
        super().__init__('json_bridge_test')
        
        # 订阅JSON格式的主题
        self.grid_map_json_sub = self.create_subscription(
            String, '/grid_map_json', self.grid_map_json_callback, 10
        )
        
        self.planning_request_json_sub = self.create_subscription(
            String, '/planning_request_json', self.planning_request_json_callback, 10
        )
        
        logger.info("JSON桥接测试节点已启动 - 正在监听JSON格式消息")
        
    def grid_map_json_callback(self, msg):
        """处理网格地图JSON消息"""
        try:
            # 解析JSON数据
            data = json.loads(msg.data)
            
            # 输出关键信息，验证接收正确
            if "data" in data:
                map_data = data["data"]
                logger.info("收到网格地图JSON消息:")
                logger.info(f"  - 类型信息: {data.get('type', '未指定')}")
                logger.info(f"  - 地图尺寸: {map_data.get('width', 0)}x{map_data.get('height', 0)}")
                logger.info(f"  - 分辨率: {map_data.get('resolution', 0.0)}")
                
                # 检查障碍物数量（假设值为100代表障碍物）
                data_array = map_data.get("data", [])
                if data_array:
                    obstacle_count = sum(1 for cell in data_array if cell == 100)
                    logger.info(f"  - 障碍物数量: {obstacle_count}")
            else:
                logger.warning("JSON消息格式不符合预期结构")
                logger.info(f"收到的原始JSON: {msg.data[:200]}...（已截断）")
                
        except json.JSONDecodeError as e:
            logger.error(f"JSON解析错误: {e}")
        except Exception as e:
            logger.error(f"处理地图JSON时出错: {e}")
            
    def planning_request_json_callback(self, msg):
        """处理规划请求JSON消息"""
        try:
            # 解析JSON数据
            data = json.loads(msg.data)
            
            # 输出关键信息，验证接收正确
            if "data" in data:
                request_data = data["data"]
                logger.info("收到规划请求JSON消息:")
                logger.info(f"  - 类型信息: {data.get('type', '未指定')}")
                logger.info(f"  - 规划器类型: {request_data.get('planner_type', 'unknown')}")
                
                # 提取起点和终点信息
                start = request_data.get("start", {})
                goal = request_data.get("goal", {})
                
                if start and "pose" in start and "position" in start.get("pose", {}):
                    start_pos = start["pose"]["position"]
                    logger.info(f"  - 起点坐标: ({start_pos.get('x', 0.0)}, {start_pos.get('y', 0.0)})")
                
                if goal and "pose" in goal and "position" in goal.get("pose", {}):
                    goal_pos = goal["pose"]["position"]
                    logger.info(f"  - 终点坐标: ({goal_pos.get('x', 0.0)}, {goal_pos.get('y', 0.0)})")
            else:
                logger.warning("JSON消息格式不符合预期结构")
                logger.info(f"收到的原始JSON: {msg.data[:200]}...（已截断）")
                
        except json.JSONDecodeError as e:
            logger.error(f"JSON解析错误: {e}")
        except Exception as e:
            logger.error(f"处理规划请求JSON时出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = JSONBridgeTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("用户中断，停止节点...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

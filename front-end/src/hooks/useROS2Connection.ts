import { useEffect, useState, useCallback, useRef } from 'react';
import type { VehicleState, Obstacle, ParkingSpot, GridMap, PlanningPath } from '../types';

// WebSocket连接状态
export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected' | 'error';

// 连接错误类型
export interface ConnectionError {
  type: 'websocket' | 'parse' | 'subscription' | 'network';
  message: string;
  timestamp: number;
}

export interface ROS2Data {
  vehicleState: VehicleState | null;
  obstacles: Obstacle[];
  parkingSpots: ParkingSpot[];
  gridMap: GridMap | null;
  planningPath: PlanningPath | null;
}

// Hook返回值类型
export interface UseROS2ConnectionReturn {
  connectionStatus: ConnectionStatus;
  data: ROS2Data;
  lastError: ConnectionError | null;
  connect: () => void;
  disconnect: () => void;
  sendPlanningRequest: (startPos: { x: number; y: number }, goalPos: { x: number; y: number }) => Promise<boolean>;
  isConnected: boolean;
  reconnectAttempts: number;
}

// 四元数转欧拉角工具函数
const quaternionToEuler = (q: { x: number; y: number; z: number; w: number }): number => {
  const { x, y, z, w } = q;
  // 计算 yaw 角度 (Z轴旋转)
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return yaw;
};

// 检查消息结构的工具函数
const safeGet = <T>(obj: unknown, path: string, defaultValue: T): T => {
  try {
    const result = path.split('.').reduce((current: unknown, key: string) => {
      return current && typeof current === 'object' && current !== null && key in current 
        ? (current as Record<string, unknown>)[key] 
        : defaultValue;
    }, obj);
    return result !== undefined && result !== null ? result as T : defaultValue;
  } catch {
    return defaultValue;
  }
};

// 验证位置数据
const validatePosition = (pos: unknown): { x: number; y: number; z: number } => {
  const position = pos as Record<string, unknown>;
  return {
    x: typeof position?.x === 'number' && !isNaN(position.x) ? position.x : 0,
    y: typeof position?.y === 'number' && !isNaN(position.y) ? position.y : 0,
    z: typeof position?.z === 'number' && !isNaN(position.z) ? position.z : 0
  };
};

// 验证四元数数据
const validateQuaternion = (q: unknown): { x: number; y: number; z: number; w: number } => {
  const quaternion = q as Record<string, unknown>;
  return {
    x: typeof quaternion?.x === 'number' && !isNaN(quaternion.x) ? quaternion.x : 0,
    y: typeof quaternion?.y === 'number' && !isNaN(quaternion.y) ? quaternion.y : 0,
    z: typeof quaternion?.z === 'number' && !isNaN(quaternion.z) ? quaternion.z : 0,
    w: typeof quaternion?.w === 'number' && !isNaN(quaternion.w) ? quaternion.w : 1
  };
};

export const useROS2Connection = (wsUrl: string = 'ws://localhost:9090'): UseROS2ConnectionReturn => {
  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>('disconnected');
  const [data, setData] = useState<ROS2Data>({
    vehicleState: null,
    obstacles: [],
    parkingSpots: [],
    gridMap: null,
    planningPath: null,
  });
  const [lastError, setLastError] = useState<ConnectionError | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const reconnectAttempts = useRef(0);
  const maxReconnectAttempts = 5;
  const isConnectedRef = useRef(false);

  // 处理ROS2消息的函数
  const handleROS2Message = useCallback((message: { topic: string; msg: unknown }) => {
    try {
      const { topic, msg } = message;

      switch (topic) {
        case '/vehicle_state': {
          const position = validatePosition(safeGet(msg, 'pose.position', {}));
          const orientation = validateQuaternion(safeGet(msg, 'pose.orientation', {}));
          
          setData(prev => ({
            ...prev,
            vehicleState: {
              position,
              orientation,
              velocity: { linear_x: 0, linear_y: 0, angular_z: 0 },
              status: 'moving'
            }
          }));
          break;
        }

        case '/detected_objects': {
          const markers = safeGet(msg, 'markers', [] as unknown[]);
          const obstacles: Obstacle[] = (markers as Array<Record<string, unknown>>).map((marker, index) => ({
            id: marker.id?.toString() || `obstacle_${index}`,
            type: marker.ns === 'detected_objects' ? 'obstacle' : 'vehicle',
            position: validatePosition(safeGet(marker, 'pose.position', {})),
            size: {
              width: safeGet(marker, 'scale.y', 1),
              length: safeGet(marker, 'scale.x', 1),
              height: safeGet(marker, 'scale.z', 1)
            },
            orientation: quaternionToEuler(validateQuaternion(safeGet(marker, 'pose.orientation', {})))
          }));
          setData(prev => ({ ...prev, obstacles }));
          break;
        }

        case '/grid_map': {
          setData(prev => ({
            ...prev,
            gridMap: {
              width: safeGet(msg, 'width', 0),
              height: safeGet(msg, 'height', 0),
              resolution: safeGet(msg, 'resolution', 0.1),
              origin: { 
                x: safeGet(msg, 'origin.position.x', 0), 
                y: safeGet(msg, 'origin.position.y', 0) 
              },
              data: safeGet(msg, 'data', [] as number[])
            }
          }));
          break;
        }

        case '/planning_path': {
          const points = safeGet(msg, 'points', [] as unknown[]);
          setData(prev => ({
            ...prev,
            planningPath: {
              points: (points as Array<Record<string, unknown>>).map((point) => ({
                position: validatePosition(safeGet(point, 'pose.pose.position', {})),
                velocity: safeGet(point, 'velocity', 0),
                curvature: safeGet(point, 'curvature', 0)
              })),
              total_length: safeGet(msg, 'total_length', 0),
              planner_type: safeGet(msg, 'planner_type', 'unknown')
            }
          }));
          break;
        }

        case '/parking_spots': {
          try {
            const parkingSpotsData = JSON.parse(safeGet(msg, 'data', '[]'));
            setData(prev => ({ 
              ...prev, 
              parkingSpots: Array.isArray(parkingSpotsData) ? parkingSpotsData : [] 
            }));
          } catch (error) {
            console.error('Error parsing parking spots data:', error);
            setData(prev => ({ ...prev, parkingSpots: [] }));
          }
          break;
        }
      }
    } catch (error) {
      console.error('Error handling ROS2 message:', error);
      setLastError({
        type: 'parse',
        message: `Failed to handle message from topic ${message.topic}: ${error}`,
        timestamp: Date.now()
      });
    }
  }, []);

  // 连接函数
  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      return;
    }

    // 清理之前的连接
    if (wsRef.current) {
      wsRef.current.close();
    }

    setConnectionStatus('connecting');
    setLastError(null);
    
    try {
      const websocket = new WebSocket(wsUrl);
      wsRef.current = websocket;

      websocket.onopen = () => {
        console.log('WebSocket connected to ROS2 bridge');
        setConnectionStatus('connected');
        isConnectedRef.current = true;
        reconnectAttempts.current = 0;
        
        // 订阅ROS2话题
        const subscriptions = [
          {
            op: 'subscribe',
            topic: '/vehicle_state',
            type: 'geometry_msgs/PoseStamped'
          },
          {
            op: 'subscribe', 
            topic: '/detected_objects',
            type: 'visualization_msgs/MarkerArray'
          },
          {
            op: 'subscribe',
            topic: '/grid_map',
            type: 'auto_msgs/GridMap'
          },
          {
            op: 'subscribe',
            topic: '/planning_path',
            type: 'auto_msgs/PlanningPath'
          },
          {
            op: 'subscribe',
            topic: '/parking_spots',
            type: 'std_msgs/String'
          }
        ];

        subscriptions.forEach(sub => {
          try {
            websocket.send(JSON.stringify(sub));
          } catch (error) {
            console.error('Error sending subscription:', error);
            setLastError({
              type: 'subscription',
              message: `Failed to subscribe to ${sub.topic}: ${error}`,
              timestamp: Date.now()
            });
          }
        });
      };

      websocket.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          handleROS2Message(message);
        } catch (error) {
          console.error('Error parsing WebSocket message:', error);
          setLastError({
            type: 'parse',
            message: `Failed to parse message: ${error}`,
            timestamp: Date.now()
          });
        }
      };

      websocket.onclose = (event) => {
        console.log('WebSocket disconnected', event.code, event.reason);
        setConnectionStatus('disconnected');
        isConnectedRef.current = false;
        wsRef.current = null;
        
        // 自动重连逻辑
        if (reconnectAttempts.current < maxReconnectAttempts && !event.wasClean) {
          reconnectAttempts.current += 1;
          const delay = Math.min(1000 * Math.pow(2, reconnectAttempts.current), 30000);
          console.log(`Attempting to reconnect in ${delay}ms (attempt ${reconnectAttempts.current}/${maxReconnectAttempts})`);
          
          reconnectTimeoutRef.current = setTimeout(() => {
            connect();
          }, delay);
        } else if (reconnectAttempts.current >= maxReconnectAttempts) {
          setLastError({
            type: 'network',
            message: `Max reconnection attempts (${maxReconnectAttempts}) exceeded`,
            timestamp: Date.now()
          });
        }
      };

      websocket.onerror = (error) => {
        console.error('WebSocket error:', error);
        setConnectionStatus('error');
        setLastError({
          type: 'websocket',
          message: 'WebSocket connection error',
          timestamp: Date.now()
        });
      };

    } catch (error) {
      console.error('Failed to create WebSocket:', error);
      setConnectionStatus('error');
      setLastError({
        type: 'websocket',
        message: `Failed to create WebSocket: ${error}`,
        timestamp: Date.now()
      });
    }
  }, [wsUrl, handleROS2Message]);

  // 发送规划请求
  const sendPlanningRequest = useCallback(async (
    startPos: { x: number; y: number }, 
    goalPos: { x: number; y: number }
  ): Promise<boolean> => {
    return new Promise((resolve) => {
      if (wsRef.current?.readyState === WebSocket.OPEN) {
        try {
          const planningRequest = {
            op: 'publish',
            topic: '/planning_request',
            msg: {
              header: {
                frame_id: 'map',
                stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 }
              },
              start: {
                pose: {
                  position: { x: startPos.x, y: startPos.y, z: 0 },
                  orientation: { x: 0, y: 0, z: 0, w: 1 }
                }
              },
              goal: {
                pose: {
                  position: { x: goalPos.x, y: goalPos.y, z: 0 },
                  orientation: { x: 0, y: 0, z: 0, w: 1 }
                }
              },
              planner_type: 'hybrid_astar',
              consider_kinematic: true
            }
          };
          
          wsRef.current.send(JSON.stringify(planningRequest));
          resolve(true);
        } catch (error) {
          console.error('Error sending planning request:', error);
          setLastError({
            type: 'websocket',
            message: `Failed to send planning request: ${error}`,
            timestamp: Date.now()
          });
          resolve(false);
        }
      } else {
        console.warn('WebSocket is not connected');
        setLastError({
          type: 'websocket',
          message: 'Cannot send planning request: WebSocket not connected',
          timestamp: Date.now()
        });
        resolve(false);
      }
    });
  }, []);

  // 断开连接
  const disconnect = useCallback(() => {
    // 清理重连定时器
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    
    // 关闭WebSocket连接
    if (wsRef.current) {
      wsRef.current.close(1000, 'Manual disconnect');
      wsRef.current = null;
    }
    
    isConnectedRef.current = false;
    setConnectionStatus('disconnected');
    reconnectAttempts.current = 0;
  }, []);

  // 清理效果
  useEffect(() => {
    return () => {
      disconnect();
    };
  }, [disconnect]);

  return {
    connectionStatus,
    data,
    lastError,
    connect,
    disconnect,
    sendPlanningRequest,
    isConnected: isConnectedRef.current && connectionStatus === 'connected',
    reconnectAttempts: reconnectAttempts.current
  };
};

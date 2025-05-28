// 类型定义
export interface VehicleState {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
  velocity: { linear_x: number; linear_y: number; angular_z: number };
  status: 'idle' | 'moving' | 'parking' | 'emergency';
}

export interface Obstacle {
  id: string;
  type: 'vehicle' | 'pedestrian' | 'obstacle' | 'building';
  position: { x: number; y: number; z: number };
  size: { width: number; length: number; height: number };
  orientation: number;
}

export interface ParkingSpot {
  id: string;
  position: { x: number; y: number };
  size: { width: number; length: number };
  status: 'available' | 'occupied' | 'reserved';
  orientation: number;
}

export interface GridMap {
  width: number;
  height: number;
  resolution: number;
  origin: { x: number; y: number };
  data: number[];
}

export interface PathPoint {
  position: { x: number; y: number; z: number };
  velocity: number;
  curvature: number;
}

export interface PlanningPath {
  points: PathPoint[];
  total_length: number;
  planner_type: string;
}

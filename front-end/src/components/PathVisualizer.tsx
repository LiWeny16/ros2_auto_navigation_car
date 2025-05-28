import React from 'react';
import { type PlanningPath } from '../types';
import { Line } from '@react-three/drei';
import * as THREE from 'three';

interface PathVisualizerProps {
  path: PlanningPath | null;
}

export const PathVisualizer: React.FC<PathVisualizerProps> = ({ path }) => {
  if (!path || path.points.length === 0) {
    return null;
  }

  // 转换路径点为Three.js坐标
  const pathPoints = path.points.map(point => 
    new THREE.Vector3(
      point.position.x,
      point.position.z + 0.1, // 稍微抬高以避免与地面重叠
      -point.position.y
    )
  );

  // 根据速度生成颜色渐变
  const getPathColor = (velocity: number, maxVelocity: number = 2.0) => {
    const ratio = Math.min(velocity / maxVelocity, 1.0);
    const r = ratio;
    const g = 1 - ratio;
    return new THREE.Color(r, g, 0);
  };

  return (
    <group>
      {/* 主路径线 */}
      <Line
        points={pathPoints}
        color="#00ff00"
        lineWidth={5}
        transparent={true}
        opacity={0.8}
      />

      {/* 路径点标记 */}
      {path.points.map((point, index) => (
        <mesh
          key={index}
          position={[
            point.position.x,
            point.position.z + 0.2,
            -point.position.y
          ]}
        >
          <sphereGeometry args={[0.1]} />
          <meshStandardMaterial 
            color={getPathColor(point.velocity)}
            emissive={getPathColor(point.velocity)}
            emissiveIntensity={0.3}
          />
        </mesh>
      ))}

      {/* 起点标记 */}
      {path.points.length > 0 && (
        <mesh
          position={[
            path.points[0].position.x,
            path.points[0].position.z + 0.5,
            -path.points[0].position.y
          ]}
        >
          <coneGeometry args={[0.3, 1.0]} />
          <meshStandardMaterial color="#00ff00" />
        </mesh>
      )}

      {/* 终点标记 */}
      {path.points.length > 0 && (
        <mesh
          position={[
            path.points[path.points.length - 1].position.x,
            path.points[path.points.length - 1].position.z + 0.5,
            -path.points[path.points.length - 1].position.y
          ]}
        >
          <coneGeometry args={[0.3, 1.0]} />
          <meshStandardMaterial color="#ff0000" />
        </mesh>
      )}

      {/* 方向箭头 */}
      {path.points.slice(0, -1).map((point, index) => {
        const nextPoint = path.points[index + 1];
        const direction = new THREE.Vector3(
          nextPoint.position.x - point.position.x,
          0,
          -(nextPoint.position.y - point.position.y)
        ).normalize();
        
        const position = new THREE.Vector3(
          point.position.x,
          point.position.z + 0.15,
          -point.position.y
        );

        return (
          <mesh
            key={`arrow-${index}`}
            position={position}
            lookAt={position.clone().add(direction)}
          >
            <coneGeometry args={[0.05, 0.2]} />
            <meshStandardMaterial color="#ffff00" />
          </mesh>
        );
      })}
    </group>
  );
};

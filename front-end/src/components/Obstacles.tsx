import React, { useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Text } from '@react-three/drei';
import { useTheme } from '@mui/material/styles';
import type { Obstacle } from '../types';
import * as THREE from 'three';

interface ObstaclesProps {
  obstacles: Obstacle[];
}

export const Obstacles: React.FC<ObstaclesProps> = ({ obstacles }) => {
  const theme = useTheme();

  const getObstacleColor = (type: string) => {
    switch (type) {
      case 'vehicle': return theme.palette.error.main; // 红色
      case 'pedestrian': return theme.palette.info.main; // 蓝色
      case 'building': return theme.palette.text.secondary; // 灰色
      default: return theme.palette.warning.main; // 橙色
    }
  };

  const getObstacleGeometry = (obstacle: Obstacle) => {
    const { width, length, height } = obstacle.size;
    
    switch (obstacle.type) {
      case 'pedestrian':
        return <cylinderGeometry args={[width/2, width/2, height]} />;
      case 'building':
        return <boxGeometry args={[length, height, width]} />;
      default:
        return <boxGeometry args={[length, height, width]} />;
    }
  };

  const ObstacleWithLabel: React.FC<{ obstacle: Obstacle }> = ({ obstacle }) => {
    const meshRef = useRef<THREE.Mesh>(null);
    
    useFrame((state) => {
      if (meshRef.current) {
        // 添加轻微的呼吸动画效果
        const time = state.clock.getElapsedTime();
        meshRef.current.scale.setScalar(1 + Math.sin(time * 2) * 0.02);
      }
    });

    return (
      <group
        position={[
          obstacle.position.x,
          obstacle.position.z + obstacle.size.height / 2,
          -obstacle.position.y
        ]}
        rotation={[0, obstacle.orientation, 0]}
      >
        <mesh ref={meshRef}>
          {getObstacleGeometry(obstacle)}
          <meshStandardMaterial 
            color={getObstacleColor(obstacle.type)}
            transparent={obstacle.type === 'building'}
            opacity={obstacle.type === 'building' ? 0.7 : 1.0}
            roughness={0.3}
            metalness={0.1}
          />
        </mesh>
        
        {/* 添加类型标签 */}
        <Text
          position={[0, obstacle.size.height / 2 + 0.5, 0]}
          fontSize={0.3}
          color={theme.palette.text.primary}
          anchorX="center"
          anchorY="middle"
        >
          {obstacle.type.toUpperCase()}
        </Text>
        
        {/* 添加警告框 */}
        <mesh
          position={[0, -obstacle.size.height / 2 - 0.1, 0]}
          rotation={[-Math.PI / 2, 0, 0]}
        >
          <ringGeometry args={[obstacle.size.width * 0.6, obstacle.size.width * 0.8]} />
          <meshBasicMaterial 
            color={getObstacleColor(obstacle.type)} 
            transparent 
            opacity={0.3}
          />
        </mesh>
      </group>
    );
  };

  return (
    <group>
      {obstacles.map((obstacle) => (
        <ObstacleWithLabel key={obstacle.id} obstacle={obstacle} />
      ))}
    </group>
  );
};

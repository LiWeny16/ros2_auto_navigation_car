import React, { useMemo } from 'react';
import * as THREE from 'three';
import { type GridMap } from '../types';

interface RoadMapProps {
  gridMap: GridMap;
}

export const RoadMap: React.FC<RoadMapProps> = ({ gridMap }) => {
  // 创建地图纹理
  const texture = useMemo(() => {
    const canvas = document.createElement('canvas');
    canvas.width = gridMap.width;
    canvas.height = gridMap.height;
    const ctx = canvas.getContext('2d');
    
    if (ctx) {
      // 绘制网格地图
      const imageData = ctx.createImageData(gridMap.width, gridMap.height);
      const data = imageData.data;
      
      for (let i = 0; i < gridMap.data.length; i++) {
        const value = gridMap.data[i];
        const pixelIndex = i * 4;
        
        // 根据地图值设置颜色
        if (value < 0) {
          // 未知区域 - 灰色
          data[pixelIndex] = 128;     // R
          data[pixelIndex + 1] = 128; // G
          data[pixelIndex + 2] = 128; // B
        } else if (value === 0) {
          // 自由空间 - 浅绿色
          data[pixelIndex] = 100;     // R
          data[pixelIndex + 1] = 200; // G
          data[pixelIndex + 2] = 100; // B
        } else {
          // 障碍物 - 深红色
          data[pixelIndex] = 200;     // R
          data[pixelIndex + 1] = 50;  // G
          data[pixelIndex + 2] = 50;  // B
        }
        data[pixelIndex + 3] = 255; // A
      }
      
      ctx.putImageData(imageData, 0, 0);
    }
    
    const texture = new THREE.CanvasTexture(canvas);
    texture.flipY = false;
    return texture;
  }, [gridMap]);

  // 地图尺寸计算
  const mapWidth = gridMap.width * gridMap.resolution;
  const mapHeight = gridMap.height * gridMap.resolution;

  return (
    <group>
      {/* 主地图平面 */}
      <mesh 
        position={[
          gridMap.origin.x + mapWidth / 2,
          0,
          -(gridMap.origin.y + mapHeight / 2)
        ]}
        rotation={[-Math.PI / 2, 0, 0]}
        receiveShadow
      >
        <planeGeometry args={[mapWidth, mapHeight]} />
        <meshStandardMaterial 
          map={texture} 
          transparent={false}
          roughness={0.8}
          metalness={0.1}
        />
      </mesh>

      {/* 网格线 */}
      <mesh 
        position={[
          gridMap.origin.x + mapWidth / 2,
          0.01,
          -(gridMap.origin.y + mapHeight / 2)
        ]}
        rotation={[-Math.PI / 2, 0, 0]}
      >
        <planeGeometry args={[mapWidth, mapHeight]} />
        <meshBasicMaterial 
          color="#ffffff" 
          transparent 
          opacity={0.1}
          wireframe
        />
      </mesh>

      {/* 坐标轴指示器 */}
      <group position={[gridMap.origin.x, 0.1, -gridMap.origin.y]}>
        {/* X轴 - 红色 */}
        <mesh position={[2.5, 0, 0]}>
          <cylinderGeometry args={[0.05, 0.05, 5]} />
          <meshBasicMaterial color="#ff0000" />
        </mesh>
        <mesh position={[5, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <coneGeometry args={[0.2, 0.5]} />
          <meshBasicMaterial color="#ff0000" />
        </mesh>

        {/* Y轴 - 绿色 */}
        <mesh position={[0, 0, -2.5]}>
          <cylinderGeometry args={[0.05, 0.05, 5]} />
          <meshBasicMaterial color="#00ff00" />
        </mesh>
        <mesh position={[0, 0, -5]} rotation={[Math.PI / 2, 0, 0]}>
          <coneGeometry args={[0.2, 0.5]} />
          <meshBasicMaterial color="#00ff00" />
        </mesh>
      </group>
    </group>
  );
};

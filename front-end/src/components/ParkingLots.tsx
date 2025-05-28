import React from 'react';
import * as THREE from 'three';
import { type ParkingSpot } from '../types';

interface ParkingLotsProps {
  parkingSpots: ParkingSpot[];
}

export const ParkingLots: React.FC<ParkingLotsProps> = ({ parkingSpots }) => {
  const getSpotColor = (status: string) => {
    switch (status) {
      case 'available': return '#00ff00';   // 绿色 - 可用
      case 'occupied': return '#ff0000';    // 红色 - 已占用
      case 'reserved': return '#ffff00';    // 黄色 - 已预订
      default: return '#888888';            // 灰色 - 未知
    }
  };

  return (
    <group>
      {parkingSpots.map((spot) => (
        <group key={spot.id}>
          {/* 停车位地面标记 */}
          <mesh
            position={[
              spot.position.x,
              0.01,
              -spot.position.y
            ]}
            rotation={[-Math.PI / 2, 0, spot.orientation]}
          >
            <planeGeometry args={[spot.size.length, spot.size.width]} />
            <meshStandardMaterial 
              color={getSpotColor(spot.status)}
              transparent={true}
              opacity={0.6}
            />
          </mesh>

          {/* 停车位边框 */}
          <lineSegments
            position={[
              spot.position.x,
              0.02,
              -spot.position.y
            ]}
            rotation={[0, spot.orientation, 0]}
          >
            <edgesGeometry 
              args={[new THREE.PlaneGeometry(spot.size.length, spot.size.width)]} 
            />
            <lineBasicMaterial color="#ffffff" linewidth={2} />
          </lineSegments>

          {/* 停车位编号 */}
          <mesh
            position={[
              spot.position.x,
              0.03,
              -spot.position.y
            ]}
            rotation={[-Math.PI / 2, 0, spot.orientation]}
          >
            <planeGeometry args={[1, 0.5]} />
            <meshStandardMaterial color="#ffffff" />
          </mesh>
        </group>
      ))}
    </group>
  );
};

import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { Text } from '@react-three/drei';
import { useTheme } from '@mui/material/styles';
import * as THREE from 'three';
import type { VehicleState } from '../types';
import { vehicleStatusColors } from '../theme';

interface VehicleProps {
  vehicleState: VehicleState;
}

export const Vehicle: React.FC<VehicleProps> = ({ vehicleState }) => {
  const theme = useTheme();
  const meshRef = useRef<THREE.Group>(null);
  const wheelRefs = useRef<THREE.Mesh[]>([]);
  
  // 计算车辆速度
  const speed = useMemo(() => {
    return Math.sqrt(
      vehicleState.velocity.linear_x ** 2 + 
      vehicleState.velocity.linear_y ** 2
    );
  }, [vehicleState.velocity]);

  useFrame((state) => {
    if (meshRef.current) {
      // 更新车辆位置
      meshRef.current.position.set(
        vehicleState.position.x,
        vehicleState.position.z,
        -vehicleState.position.y
      );
      
      // 更新车辆朝向（从四元数转换为欧拉角）
      const quaternion = new THREE.Quaternion(
        vehicleState.orientation.x,
        vehicleState.orientation.y,
        vehicleState.orientation.z,
        vehicleState.orientation.w
      );
      meshRef.current.setRotationFromQuaternion(quaternion);
      
      // 车轮转动动画
      const time = state.clock.getElapsedTime();
      wheelRefs.current.forEach((wheel) => {
        if (wheel) {
          wheel.rotation.x = time * speed * 2;
        }
      });
    }
  });

  // 根据状态改变颜色
  const getVehicleColor = () => {
    return vehicleStatusColors[vehicleState.status] || theme.palette.primary.main;
  };

  const getStatusText = () => {
    const statusMap = {
      idle: '待机',
      moving: '行驶中',
      parking: '停车中',
      emergency: '紧急'
    };
    return statusMap[vehicleState.status] || '未知';
  };

  return (
    <group ref={meshRef}>
      {/* 车身 */}
      <mesh position={[0, 0.75, 0]} castShadow>
        <boxGeometry args={[4.8, 1.5, 2.2]} />
        <meshStandardMaterial 
          color={getVehicleColor()} 
          roughness={0.3}
          metalness={0.7}
        />
      </mesh>
      
      {/* 车顶 */}
      <mesh position={[0, 1.8, 0]} castShadow>
        <boxGeometry args={[3.0, 0.6, 1.8]} />
        <meshStandardMaterial 
          color={theme.palette.text.secondary} 
          roughness={0.2}
          metalness={0.8}
        />
      </mesh>
      
      {/* 前车灯 */}
      <mesh position={[2.2, 0.5, 0.8]} castShadow>
        <sphereGeometry args={[0.2]} />
        <meshStandardMaterial 
          color={theme.palette.text.primary} 
          emissive={theme.palette.text.primary} 
          emissiveIntensity={vehicleState.status === 'moving' ? 0.5 : 0.2}
        />
      </mesh>
      <mesh position={[2.2, 0.5, -0.8]} castShadow>
        <sphereGeometry args={[0.2]} />
        <meshStandardMaterial 
          color={theme.palette.text.primary} 
          emissive={theme.palette.text.primary} 
          emissiveIntensity={vehicleState.status === 'moving' ? 0.5 : 0.2}
        />
      </mesh>
      
      {/* 尾灯 */}
      <mesh position={[-2.2, 0.5, 0.8]} castShadow>
        <sphereGeometry args={[0.15]} />
        <meshStandardMaterial 
          color={theme.palette.error.main} 
          emissive={theme.palette.error.main}
          emissiveIntensity={vehicleState.status === 'emergency' ? 1.0 : 0.3}
        />
      </mesh>
      <mesh position={[-2.2, 0.5, -0.8]} castShadow>
        <sphereGeometry args={[0.15]} />
        <meshStandardMaterial 
          color={theme.palette.error.main} 
          emissive={theme.palette.error.main}
          emissiveIntensity={vehicleState.status === 'emergency' ? 1.0 : 0.3}
        />
      </mesh>

      {/* 车轮 */}
      {[[-1.5, -0.3, 1.2], [1.5, -0.3, 1.2], [-1.5, -0.3, -1.2], [1.5, -0.3, -1.2]].map((pos, i) => (
        <mesh 
          key={i} 
          position={pos as [number, number, number]}
          ref={(el) => {
            if (el) wheelRefs.current[i] = el;
          }}
          castShadow
        >
          <cylinderGeometry args={[0.4, 0.4, 0.3]} />
          <meshStandardMaterial color={theme.palette.text.secondary} roughness={0.8} />
        </mesh>
      ))}

      {/* 状态指示器 */}
      <mesh position={[0, 3, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[0.5]} />
        <meshBasicMaterial 
          color={getVehicleColor()} 
          transparent 
          opacity={0.8}
        />
      </mesh>

      {/* 状态文本 */}
      <Text
        position={[0, 3.5, 0]}
        fontSize={0.4}
        color="#ffffff"
        anchorX="center"
        anchorY="middle"
        font="/fonts/Inter-Regular.woff"
      >
        {getStatusText()}
      </Text>

      {/* 速度指示 */}
      <Text
        position={[0, 2.8, 0]}
        fontSize={0.3}
        color="#00ff00"
        anchorX="center"
        anchorY="middle"
        font="/fonts/Inter-Regular.woff"
      >
        {`${speed.toFixed(1)} m/s`}
      </Text>

      {/* 车辆轮廓线 */}
      <mesh position={[0, 0, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[3.5, 4]} />
        <meshBasicMaterial 
          color={getVehicleColor()} 
          transparent 
          opacity={0.3}
        />
      </mesh>
    </group>
  );
};

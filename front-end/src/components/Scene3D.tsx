import React, { Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Sky } from '@react-three/drei';
import type { SxProps, Theme } from '@mui/material';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Button,
  Chip,
  Stack,
  Divider,
  ThemeProvider,
  CircularProgress,
} from '@mui/material';
import {
  Circle as CircleIcon,
  DirectionsCar as CarIcon,
  Traffic as TrafficIcon,
  LocalParking as ParkingIcon,
  Timeline as PathIcon,
  Speed as SpeedIcon,
  LocationOn as LocationIcon
} from '@mui/icons-material';
import { Vehicle } from './Vehicle';
import { Obstacles } from './Obstacles';
import { RoadMap } from './RoadMap';
import { ParkingLots } from './ParkingLots';
import { PathVisualizer } from './PathVisualizer';
import { useROS2Connection } from '../hooks/useROS2Connection';
import { theme, connectionStatusColors, vehicleStatusColors } from '../theme';
import { CustomAlert } from './common/CustomAlert';

const Scene3D: React.FC = () => {
  const { connectionStatus, data, connect, disconnect, sendPlanningRequest, isConnected, lastError } = useROS2Connection();

  const handleMapClick = (event: { point?: { x: number; z: number } }) => {
    if (event.point) {
      const { x, z } = event.point;
      const startPos = data.vehicleState ?
        { x: data.vehicleState.position.x, y: data.vehicleState.position.y } :
        { x: 0, y: 0 };
      const goalPos = { x, y: -z };

      sendPlanningRequest(startPos, goalPos);
    }
  };

  const getConnectionStatusChip = () => {
    const color = connectionStatusColors[connectionStatus];
    const labels = {
      connecting: '连接中...',
      connected: '已连接',
      disconnected: '未连接',
      error: '连接错误'
    };

    return (
      <Chip
        icon={<CircleIcon />}
        label={labels[connectionStatus]}
        sx={{
          backgroundColor: color,
          color: 'white',
          fontWeight: 'bold'
        }}
      />
    );
  };

  const getVehicleStatusChip = () => {
    if (!data.vehicleState) return null;

    const color = vehicleStatusColors[data.vehicleState.status];
    const labels = {
      idle: '待机',
      moving: '行驶中',
      parking: '停车中',
      emergency: '紧急状态'
    };

    return (
      <Chip
        icon={<CarIcon />}
        label={labels[data.vehicleState.status]}
        sx={{
          backgroundColor: color,
          color: 'white',
          fontWeight: 'bold'
        }}
      />
    );
  };

  const boxStyle: SxProps<Theme> = {
    display: 'flex',
    alignItems: 'center',
    gap: 1
  };

  return (
    <ThemeProvider theme={theme}>
      <Box sx={{
        width: '100%',
        height: '100vh',
        position: 'relative',
        backgroundColor: theme.palette.background.default,
        display: 'flex'
      }}>
        {/* 左侧状态面板 */}
        <Box sx={{
          position: 'absolute',
          top: 16,
          left: 16,
          zIndex: 1000,
          display: 'flex',
          flexDirection: 'column',
          gap: 2,
          minWidth: 300
        }}>
          {/* 连接状态卡片 */}
          <Card>
            <CardContent>
              <Stack spacing={2}>
                <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
                  <Typography variant="h6" sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                    <CircleIcon color="primary" />
                    系统状态
                  </Typography>
                  {getConnectionStatusChip()}
                </Box>

                <Stack spacing={1}>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                    <Typography variant="body2" color="text.secondary">车辆状态:</Typography>
                    {getVehicleStatusChip() || <Typography variant="body2">未连接</Typography>}
                  </Box>

                  <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                    <Typography variant="body2" color="text.secondary">障碍物:</Typography>
                    <Chip
                      icon={<TrafficIcon />}
                      label={data.obstacles.length}
                      size="small"
                      color="info"
                    />
                  </Box>

                  <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                    <Typography variant="body2" color="text.secondary">停车位:</Typography>
                    <Chip
                      icon={<ParkingIcon />}
                      label={data.parkingSpots.length}
                      size="small"
                      color="info"
                    />
                  </Box>
                </Stack>

                <Divider />

                <Box sx={{ display: 'flex', gap: 1 }}>
                  {!isConnected ? (
                    <Button
                      variant="contained"
                      color="primary"
                      onClick={connect}
                      disabled={connectionStatus === 'connecting'}
                      startIcon={connectionStatus === 'connecting' ? <CircularProgress size={16} /> : <CircleIcon />}
                      fullWidth
                    >
                      {connectionStatus === 'connecting' ? '连接中...' : '连接ROS2'}
                    </Button>
                  ) : (
                    <Button
                      variant="outlined"
                      color="error"
                      onClick={disconnect}
                      startIcon={<CircleIcon />}
                      fullWidth
                    >
                      断开连接
                    </Button>
                  )}
                </Box>
              </Stack>
            </CardContent>
          </Card>

          {/* 车辆信息卡片 */}
          {data.vehicleState && (
            <Card>
              <CardContent>
                <Typography variant="h6" sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                  <CarIcon color="primary" />
                  车辆信息
                </Typography>

                <Stack spacing={1.5}>
                  <Box sx={boxStyle}>
                    <LocationIcon fontSize="small" color="secondary" />
                    <Typography variant="body2" color="text.secondary">位置:</Typography>
                    <Typography variant="body2" fontFamily="monospace">
                      ({data.vehicleState.position.x.toFixed(2)}, {data.vehicleState.position.y.toFixed(2)})
                    </Typography>
                  </Box>

                  <Box sx={boxStyle}>
                    <SpeedIcon fontSize="small" color="secondary" />
                    <Typography variant="body2" color="text.secondary">速度:</Typography>
                    <Typography variant="body2" fontFamily="monospace">
                      {Math.sqrt(
                        data.vehicleState.velocity.linear_x ** 2 +
                        data.vehicleState.velocity.linear_y ** 2
                      ).toFixed(2)} m/s
                    </Typography>
                  </Box>
                </Stack>
              </CardContent>
            </Card>
          )}

          {/* 路径信息卡片 */}
          {data.planningPath && (
            <Card>
              <CardContent>
                <Typography variant="h6" sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                  <PathIcon color="primary" />
                  路径信息
                </Typography>

                <Stack spacing={1.5}>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                    <Typography variant="body2" color="text.secondary">路径点数:</Typography>
                    <Chip label={data.planningPath.points.length} size="small" />
                  </Box>

                  <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                    <Typography variant="body2" color="text.secondary">总长度:</Typography>
                    <Typography variant="body2" fontFamily="monospace">
                      {data.planningPath.total_length.toFixed(2)}m
                    </Typography>
                  </Box>

                  <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                    <Typography variant="body2" color="text.secondary">规划器:</Typography>
                    <Chip
                      label={data.planningPath.planner_type}
                      size="small"
                      color="secondary"
                    />
                  </Box>
                </Stack>
              </CardContent>
            </Card>
          )}

          {/* 错误提示 */}
          {lastError && (
            <CustomAlert
              severity="error"
              onClose={() => {/* 可以添加清除错误的逻辑 */ }}
              sx={{ maxWidth: 300 }}
              variant="filled"
            >
              <Typography>{lastError.message}</Typography>
            </CustomAlert>
          )}

          {/* 操作提示 */}
          <Card>
            <CardContent>
              <Typography variant="body2" color="text.secondary" align="center">
                💡 点击3D地图设置目标点
              </Typography>
            </CardContent>
          </Card>
        </Box>

        {/* 3D场景 */}
        <Canvas
          camera={{
            position: [30, 40, 30],
            fov: 60
          }}
          shadows
          style={{ width: '100%', height: '100%' }}
        >
          <Suspense fallback={null}>
            {/* 环境光照 */}
            <ambientLight intensity={0.5} />
            <directionalLight
              position={[50, 50, 25]}
              intensity={1.2}
              castShadow
              shadow-mapSize-width={2048}
              shadow-mapSize-height={2048}
            />

            {/* 天空盒 */}
            <Sky sunPosition={[100, 20, 100]} />

            {/* 基础背景色 */}
            <color attach="background" args={['#87CEEB']} />

            {/* 相机控制 */}
            <OrbitControls
              enablePan={true}
              enableZoom={true}
              enableRotate={true}
              maxPolarAngle={Math.PI / 2}
              minPolarAngle={0}
              target={[0, 0, 0]}
            />

            {/* 地图和道路 */}
            {data.gridMap && (
              <group onClick={handleMapClick}>
                <RoadMap gridMap={data.gridMap} />
              </group>
            )}

            {/* 车辆 */}
            {data.vehicleState && (
              <Vehicle vehicleState={data.vehicleState} />
            )}

            {/* 障碍物 */}
            <Obstacles obstacles={data.obstacles} />

            {/* 停车场 */}
            <ParkingLots parkingSpots={data.parkingSpots} />

            {/* 路径可视化 */}
            <PathVisualizer path={data.planningPath} />

            {/* 坐标轴 */}
            <axesHelper args={[5]} />
          </Suspense>
        </Canvas>
      </Box>
    </ThemeProvider>
  );
};

export default Scene3D;
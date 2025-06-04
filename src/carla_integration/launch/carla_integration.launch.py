#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time if true'
    )
    
    declare_carla_host = DeclareLaunchArgument(
        'carla_host', default_value='localhost',
        description='CARLA server host'
    )
    
    declare_carla_port = DeclareLaunchArgument(
        'carla_port', default_value='2000',
        description='CARLA server port'
    )
    
    declare_role_name = DeclareLaunchArgument(
        'role_name', default_value='ego_vehicle',
        description='Vehicle role name'
    )
    
    declare_spawn_point = DeclareLaunchArgument(
        'spawn_point', default_value='',
        description='Vehicle spawn point'
    )
    
    # 获取包路径
    carla_integration_share = FindPackageShare('carla_integration')
    carla_config_file = PathJoinSubstitution([
        carla_integration_share, 'config', 'carla_params.yaml'
    ])
    
    # CARLA ROS Bridge 节点
    carla_ros_bridge = Node(
        package='carla_ros_bridge',
        executable='bridge',
        name='carla_ros_bridge',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'host': LaunchConfiguration('carla_host'),
                'port': LaunchConfiguration('carla_port'),
                'town': 'Town01',
                'timeout': 10.0,
                'synchronous_mode': True,
                'fixed_delta_seconds': 0.05,
            }
        ],
        output='screen'
    )
    
    # 车辆生成节点
    carla_spawn_objects = Node(
        package='carla_spawn_objects',
        executable='carla_spawn_objects',
        name='carla_spawn_objects',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'objects_definition_file': PathJoinSubstitution([
                    carla_integration_share, 'config', 'objects.json'
                ])
            }
        ],
        output='screen'
    )
    
    # 自车生成节点
    carla_ego_vehicle = Node(
        package='carla_spawn_objects',
        executable='carla_spawn_objects',
        name='carla_ego_vehicle',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'role_name': LaunchConfiguration('role_name'),
                'spawn_point': LaunchConfiguration('spawn_point'),
            }
        ],
        output='screen'
    )
    
    # CARLA 地图适配器
    carla_map_adapter = Node(
        package='carla_integration',
        executable='carla_map_adapter',
        name='carla_map_adapter',
        parameters=[carla_config_file],
        output='screen'
    )
    
    # CARLA 传感器适配器
    carla_sensor_adapter = Node(
        package='carla_integration',
        executable='carla_sensor_adapter',
        name='carla_sensor_adapter',
        parameters=[carla_config_file],
        output='screen'
    )
    
    # CARLA 控制适配器
    carla_control_adapter = Node(
        package='carla_integration',
        executable='carla_control_adapter',
        name='carla_control_adapter',
        parameters=[carla_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_carla_host,
        declare_carla_port,
        declare_role_name,
        declare_spawn_point,
        
        carla_ros_bridge,
        carla_spawn_objects,
        carla_ego_vehicle,
        carla_map_adapter,
        carla_sensor_adapter,
        carla_control_adapter,
    ])

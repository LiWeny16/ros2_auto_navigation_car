#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time if true'
    )
    
    # 获取包路径
    carla_integration_share = FindPackageShare('carla_integration')
    carla_config_file = PathJoinSubstitution([
        carla_integration_share, 'config', 'carla_params.yaml'
    ])
    
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
        carla_sensor_adapter,
        carla_control_adapter,
    ])

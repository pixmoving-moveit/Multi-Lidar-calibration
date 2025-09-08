#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='config.json',
        description='Configuration JSON file name'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file', 
        default_value='sensor_kit_calibration.yaml',
        description='Sensor calibration YAML file name'
    )

    # 获取包的share目录
    package_share_directory = get_package_share_directory('lidar_calibration_verif')
    
    # 构建配置文件的完整路径
    config_file_path = os.path.join(package_share_directory, 'config', 'config.json')
    calibration_file_path = os.path.join(package_share_directory, 'config', 'sensor_kit_calibration.yaml')

    # 创建节点
    lidar_calibration_verif_node = Node(
        package='lidar_calibration_verif',
        executable='lidar_calibration_verif',
        name='lidar_calibration_verif_node',
        parameters=[{
            'config_file_path': config_file_path,
            'calibration_file_path': calibration_file_path
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        calibration_file_arg,
        lidar_calibration_verif_node
    ])

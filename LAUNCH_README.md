# 激光雷达标定验证工具

## 概述
该工具用于验证激光雷达的标定参数，支持多个激光雷达传感器的点云数据处理和可视化。

## 文件结构
```
lidar_calibration_verif/
├── launch/                                    # Launch文件目录
│   ├── lidar_calibration_verif.launch.py    # 主launch文件（可配置参数）
│   └── lidar_calibration_verif_simple.launch.py # 简单launch文件（默认配置）
├── config/                                   # 配置文件目录
│   ├── config.json                          # 主配置文件
│   └── sensor_kit_calibration.yaml         # 传感器标定文件
├── src/                                     # 源代码目录
├── include/                                 # 头文件目录
└── ui/                                      # Qt界面文件
```

## 使用方法

### 1. 编译包
```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select lidar_calibration_verif
source install/setup.bash
```

### 2. 启动应用

#### 方法一：使用简单launch文件（推荐）
```bash
ros2 launch lidar_calibration_verif lidar_calibration_verif_simple.launch.py
```

#### 方法二：使用可配置的launch文件
```bash
# 使用默认配置文件
ros2 launch lidar_calibration_verif lidar_calibration_verif.launch.py

# 指定特定的配置文件
ros2 launch lidar_calibration_verif lidar_calibration_verif.launch.py config_file:=my_config.json calibration_file:=my_calibration.yaml
```

#### 方法三：直接运行可执行文件
```bash
ros2 run lidar_calibration_verif lidar_calibration_verif
```

## 配置文件说明

### 1. config.json
主配置文件，包含：
- 传感器数量和索引
- 激光雷达话题名称
- 传感器框架ID
- 其他处理参数

### 2. sensor_kit_calibration.yaml
传感器标定文件，包含各个激光雷达相对于base_link的变换关系：
- 位置偏移 (x, y, z)
- 旋转角度 (roll, pitch, yaw)

## 配置文件路径管理

### 相对路径优势
- **便于代码迁移**：不依赖具体的安装路径
- **自动解析**：利用ROS2的包管理机制自动找到配置文件
- **标准化部署**：符合ROS2包的标准结构

### 路径解析机制
1. Launch文件通过`get_package_share_directory()`获取包的安装路径
2. 将配置文件路径通过ROS2参数传递给节点
3. 节点优先使用参数中的路径，如果参数为空则使用默认的相对路径

## 订阅的话题
- `/sensing/lidar/front_top/points` - 前上激光雷达点云
- `/sensing/lidar/front_left/points` - 前左激光雷达点云
- `/sensing/lidar/front_right/points` - 前右激光雷达点云
- `/sensing/lidar/rear_top/points` - 后上激光雷达点云
- `/sensing/lidar/rear_center/points` - 后中激光雷达点云

## 发布的话题
- `filtered_points_front_right` - 滤波后的点云数据
- `calibration_boxes` - 标定框可视化标记
- 静态TF变换关系

## 依赖项
- ROS2 Humble
- Qt5
- PCL (Point Cloud Library)
- Eigen3
- tf2
- yaml-cpp
- ament_index_cpp

## 故障排除

### 1. 配置文件未找到
确保配置文件存在于`config/`目录中，并且在编译时正确安装。

### 2. 节点启动失败
检查所有依赖项是否正确安装，并且ROS2环境已正确设置。

### 3. 点云数据未接收
确认激光雷达数据源正在发布，并且话题名称与配置文件中一致。

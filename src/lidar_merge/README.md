# Lidar Merge

## 概述

`lidar_merge` 是RoboMaster项目的激光雷达合并包，负责处理多个激光雷达的数据，包括点云合并、监控和同步。该包包含两个主要子包：`lidar_monitor` 和 `pointcloud_merge`。

## 功能特性

- 多激光雷达数据合并
- 激光雷达状态监控
- 点云数据同步
- 实时数据处理
- 故障检测和恢复

## 子包结构

### 1. lidar_monitor
激光雷达监控包，负责：
- 激光雷达状态监控
- 数据质量检测
- 故障诊断
- 状态报告

### 2. pointcloud_merge
点云合并包，负责：
- 多激光雷达点云合并
- 坐标系变换
- 时间同步
- 数据滤波

## 使用方法

### 1. 编译

```bash
# 编译整个工作空间
colcon build

# 或者只编译这个包
colcon build --packages-select lidar_merge
```

### 2. 启动监控

```bash
# 启动激光雷达监控
ros2 launch lidar_monitor lidar_monitor.launch.py
```

### 3. 启动合并

```bash
# 启动点云合并
ros2 launch pointcloud_merge pointcloud_merge.launch.py
```

## 话题接口

### 订阅的话题

- `/lidar1/scan`: 激光雷达1数据
- `/lidar2/scan`: 激光雷达2数据
- `/lidar3/scan`: 激光雷达3数据
- `/tf`: 坐标变换信息
- `/tf_static`: 静态坐标变换信息

### 发布的话题

- `/merged_scan`: 合并后的激光雷达数据
- `/lidar_status`: 激光雷达状态信息
- `/pointcloud_merged`: 合并后的点云数据

## 配置参数

请在 `robot_bring_up/config/sentry.yaml` 中修改以下参数：

### 激光雷达监控参数

- `lidar_topic_1/2`: 激光雷达话题（默认: "/livox/lidar_192_168_1_187", "/livox/lidar_192_168_1_104"）
- `imu_topic_1/2`: IMU话题（默认: "/livox/imu_192_168_1_187", "/livox/imu_192_168_1_104"）
- `livox_frame_id_1/2`: 激光雷达坐标系（默认: "livox_192_168_1_187", "livox_192_168_1_104"）
- `output_lidar_topic`: 输出激光雷达话题（默认: "/livox/lidar"）
- `output_imu_topic`: 输出IMU话题（默认: "/livox/imu"）
- `lidar_1/2_dog`: 看门狗阈值（默认: 20）
- `frequency`: 处理频率（默认: 50.0 Hz）

### 点云合并参数

- `OutPut_lidar_topic`: 输出点云话题（默认: "/livox/lidar"）
- `use_voxel_grid_filter`: 是否使用体素滤波（默认: true）
- `voxel_leaf_size`: 体素滤波大小（默认: 0.13）
- `main_livox_frame_id`: 主激光雷达坐标系（默认: "livox_192_168_1_104"）

## 文件结构

```
lidar_merge/
├── lidar_monitor/           # 激光雷达监控包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/
│   ├── src/
│   └── launch/
└── pointcloud_merge/        # 点云合并包
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/
    ├── src/
    ├── launch/
    └── config/
```

## 算法流程

### 1. 数据监控
```
激光雷达数据 → 质量检测 → 状态评估 → 故障诊断
```

### 2. 数据合并
```
多激光雷达数据 → 时间同步 → 坐标系变换 → 点云合并
```

### 3. 数据滤波
```
合并后点云 → 体素滤波 → 离群点滤波 → 输出点云
```

## 性能优化

### 1. 处理速度优化

请在 `robot_bring_up/config/sentry.yaml` 中修改以下参数：

```yaml
# 增加体素滤波大小
voxel_leaf_size: 0.2

# 减少处理频率
frequency: 30.0
```

### 2. 内存优化

```yaml
# 调整看门狗阈值
lidar_1_dog: 30
lidar_2_dog: 30
```

### 3. 精度优化

```yaml
# 调整体素滤波大小
voxel_leaf_size: 0.05

# 增加处理频率
frequency: 100.0
```

## 集成使用

### 与Navigation2集成

1. 在costmap配置中使用合并后的激光雷达数据
2. 配置相应的坐标系变换
3. 调整costmap参数以适应合并数据

### 与SLAM集成

1. 使用合并后的点云进行建图
2. 配置多激光雷达的TF变换
3. 调整SLAM参数

## 扩展

### 添加新的激光雷达

1. 在配置文件中添加新的激光雷达参数
2. 更新launch文件中的话题订阅
3. 添加相应的坐标系变换
4. 测试新激光雷达的集成

### 支持新的合并算法

1. 在点云合并模块中实现新算法
2. 添加相应的参数配置
3. 更新launch文件选项
4. 测试新算法的效果

## 维护者

- 维护者: cmyhj
- 邮箱: autism2484684043@163.com

## 许可证

TODO: License declaration 
# 思玄战队2025导航包
思天地之玄变，感吾生之有涯

## 简介
本导航包基于 ROS2 Humble 和 Navigation2 框架开发，参考深北理北极熊战队https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/README.md 导航包修改。

## 使用说明
1. **环境配置**
  - ROS2-Humble 推荐使用：'wget http://fishros.com/install -O fishros && . fishros'
2. **依赖安装**:
  - small_gicp点云库
     ```bash
     sudo apt install -y libeigen3-dev libomp-dev
     git clone https://github.com/koide3/small_gicp.git
     cd small_gicp
     mkdir build && cd build
     cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
     sudo make install
     cd ..
     rm -rf ./small_gicp
     ```
3. **部署代码**
```bash
mkdir -p ros_ws /
cd ~/ros_ws
```
```bash
git clone --recursive https://github.com/Abcd2024abc/SX_RM_Navigation.git src/SX_RM_Navigation
```
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 1. 地图代价扩展模块（`map_cost_extend`）
- **功能**: 提供基于点云强度的代价地图扩展功能。
- **关键文件**:
  - `src/layers/intensity_voxel_layer.cpp`: 实现了强度体素层的边界更新逻辑。
  - `src/behaviors/back_up_free_space.cpp`: 实现了自由空间的后退行为。
### 2. 路径跟踪控制器（`path_tracking_controller`）
- **功能**: 提供基于 PID 的路径跟踪控制器。
- **关键文件**:
  - `src/path_tracking_controller.cpp`: 包含控制器的配置和运行逻辑。
  - `src/pid.cpp`: 实现了 PID 控制算法。

### 3. 点云处理模块（`point_lio`）
- **功能**: 提供激光雷达点云的预处理、配准和地图构建功能。
- **关键文件**:
  - `src/laserMapping.cpp`: 实现了激光雷达点云的映射逻辑。
  - `src/li_initialization.cpp`: 处理激光雷达和 IMU 数据的初始化。

### 4. Livox 激光雷达驱动（`livox_ros_driver2`）
- **功能**: 提供 Livox 激光雷达的数据采集和发布功能。
- **关键文件**:
  - `src/comm/comm.cpp`: 包含网络通信相关的工具函数。
  - `src/parse_cfg_file/parse_cfg_file.cpp`: 解析激光雷达配置文件。

### 5. 地形分析模块（`terrain_analysis_ext`）
- **功能**: 提供地形分析和高程估计功能。
- **关键文件**:
  - `src/terrainAnalysisExt.cpp`: 实现了地形点云的处理和发布。

### 6. 小型 GICP 重定位模块（`small_gicp_relocalization`）
- **功能**: 提供基于 GICP 的机器人重定位功能。
- **关键文件**:
  - `src/small_gicp_relocalization.cpp`: 实现了重定位逻辑和 TF 广播。

### 7. 假速度变换模块（`fake_vel_transform`）
- **功能**: 提供速度变换的模拟功能。
- **关键文件**:
  - `src/fake_vel_transform.cpp`: 实现了速度变换的回调函数。

## 使用说明
1. **依赖安装**:
   - 确保已安装 ROS2 Humble 和相关依赖包。
   - 根据模块需求安装额外的依赖（如 PCL、Eigen 等）。

2. **编译项目**:
   ```bash
   colcon build --symlink-install
   ```

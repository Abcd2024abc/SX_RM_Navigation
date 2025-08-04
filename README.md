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

## 使用说明
1. **依赖安装**:
   - 确保已安装 ROS2 Humble 和相关依赖包。
   - 根据模块需求安装额外的依赖（如 PCL、Eigen 等）。

2. **编译项目**:
   ```bash
   colcon build --symlink-install
   ```

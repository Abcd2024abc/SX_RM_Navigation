# 思玄战队2025导航包
思天地之玄变，感吾生之有涯

## 简介
本导航包基于 ROS2 Humble 和 Navigation2 框架开发，参考深北理北极熊战队https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/blob/main/README.md 导航包修改。

## 部署说明
1. **环境配置**
  - ROS2-Humble 推荐使用小鱼ROS：
    '''bash
    wget http://fishros.com/install -O fishros && . fishros
    '''
2. **依赖安装**
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
4. **额外说明**
   - 本仓库使用的模拟工具为北极熊战队开发


## 使用说明
1. **依赖安装**:
   - 确保已安装 ROS2 Humble 和相关依赖包。
   - 根据模块需求安装额外的依赖（如 PCL、Eigen 等）。

2. **项目说明**:
   -Decision_tree为决策树部分，基于DQL算法开发，主要作用是为导航提供行为决策，以下为文件结构：
   '''bash
   decision_tree/
   ├── __init__.py
   ├── decision_tree.py          # 决策树主逻辑
   ├── exploration.py            # 探索策略
   ├── game_logic.py             # 游戏逻辑
   ├── navigation.py             # 导航相关功能
   ├── q_learning.py             # Q表处理
   ├── state_processor.py        # 奖励处理
   ├── utils.py                  # 工具函数
   └── verify_connection.py      # 连接验证
   '''
   -Path_Navigation为导航实现部分

   -水平有限，如有不足请见谅
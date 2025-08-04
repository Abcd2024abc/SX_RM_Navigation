#include "map_cost_extend/layers/intensity_voxel_layer.hpp"

#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#define VOXEL_BITS 16

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

using nav2_costmap_2d::Observation;
using nav2_costmap_2d::ObservationBuffer;

namespace pb_nav2_costmap_2d
{

// 初始化层
void IntensityVoxelLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in IntensityVoxelLayer::onInitialize");
  }
  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();
  footprint_clearing_enabled_ =
    node->get_parameter(name_ + ".footprint_clearing_enabled").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  max_obstacle_height_ = node->get_parameter(name_ + ".max_obstacle_height").as_double();
  combination_method_ = node->get_parameter(name_ + ".combination_method").as_int();

  // 声明参数并设置默认值
  size_z_ = node->declare_parameter(name_ + ".z_voxels", 16);
  origin_z_ = node->declare_parameter(name_ + ".origin_z", 16.0);
  min_obstacle_intensity_ = node->declare_parameter(name_ + ".min_obstacle_intensity", 0.1);
  max_obstacle_intensity_ = node->declare_parameter(name_ + ".max_obstacle_intensity", 2.0);
  z_resolution_ = node->declare_parameter(name_ + ".z_resolution", 0.05);
  unknown_threshold_ =
    node->declare_parameter(name_ + ".unknown_threshold", 15) + (VOXEL_BITS - size_z_);
  mark_threshold_ = node->declare_parameter(name_ + ".mark_threshold", 0);
  publish_voxel_ = node->declare_parameter(name_ + ".publish_voxel_map", false);

  // 如果启用了体素网格发布，创建发布者
  if (publish_voxel_) {
    voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>("voxel_grid", 1);
  }

  matchSize();
}

IntensityVoxelLayer::~IntensityVoxelLayer() {}

void IntensityVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (auto & i : transformed_footprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}

void IntensityVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
}

void IntensityVoxelLayer::reset()
{
  ObstacleLayer::reset();
  resetMaps();
}

void IntensityVoxelLayer::resetMaps()
{
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
}

// 更新边界
void IntensityVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  // 更新滚动 CostMap 发布的来源信息
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // reset 映射每次迭代
  resetMaps();

  // 如果未启用，请在此处停止
  if (!enabled_) {
    return;
  }

  // 获取作所需的最大窗口大小
  useExtraBounds(min_x, min_y, max_x, max_y);

  // 获取标记观察结果
  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;

  // 更新全局当前状态
  current_ = current;

  // 遍历所有观测点云
  for (const auto & obs : observations) {
    // 修复点云数据访问
    const unsigned char* data_ptr = obs.cloud_->data.data();
    const float* it_x = reinterpret_cast<const float*>(data_ptr + obs.cloud_->fields[0].offset);
    const float* it_y = reinterpret_cast<const float*>(data_ptr + obs.cloud_->fields[1].offset);
    const float* it_z = reinterpret_cast<const float*>(data_ptr + obs.cloud_->fields[2].offset);
    const float* it_i = reinterpret_cast<const float*>(data_ptr + obs.cloud_->fields[3].offset);

    // 缓存最大和最小障碍物距离的平方值，避免重复计算
    const double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    const double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    // 获取点云步长和大小
    const size_t point_step = obs.cloud_->point_step / sizeof(float);
    const size_t cloud_size = obs.cloud_->width * obs.cloud_->height;

    // 遍历点云中的每个点
    for (size_t i = 0; i < cloud_size; ++i) {
      // 获取点坐标和强度
      const double px = it_x[i * point_step], py = it_y[i * point_step], pz = it_z[i * point_step];
      const double intensity = it_i[i * point_step];

      // 检查点的高度和强度是否在有效范围内
      if (pz < min_obstacle_height_ || pz > max_obstacle_height_ ||
          intensity < min_obstacle_intensity_ || intensity > max_obstacle_intensity_) {
        continue;  // 如果点不符合条件，则跳过
      }

      // 计算点到传感器原点的距离的平方
      const double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                             (py - obs.origin_.y) * (py - obs.origin_.y) +
                             (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // 检查距离是否在有效范围内
      if (sq_dist <= sq_obstacle_min_range || sq_dist >= sq_obstacle_max_range) {
        continue;  // 如果距离不符合条件，则跳过
      }

      // 将世界坐标转换为地图坐标
      unsigned int mx, my, mz;
      if (!worldToMap3D(px, py, pz, mx, my, mz)) {
        continue;  // 如果转换失败，则跳过
      }

      // 在体素网格中标记体素，并检查是否需要在 costmap 中标记
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;  // 在 costmap 中标记为致命障碍
        touch(static_cast<double>(px), static_cast<double>(py), min_x, min_y, max_x,
              max_y);  // 更新边界
      }
    }
  }

  // 如果启用了体素网格发布，发布体素网格消息
  if (publish_voxel_) {
    nav2_msgs::msg::VoxelGrid grid_msg;
    grid_msg.header.stamp = clock_->now();
    grid_msg.header.frame_id = global_frame_;
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;
    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;

    const size_t data_size = grid_msg.size_x * grid_msg.size_y;
    grid_msg.data.resize(data_size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), data_size * sizeof(unsigned int));

    voxel_pub_->publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void IntensityVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // 将新原点投影到网格中
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // 使用适当的世界坐标更新 Origin
  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

}  // 命名空间 pb_nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
// 注册插件
PLUGINLIB_EXPORT_CLASS(pb_nav2_costmap_2d::IntensityVoxelLayer, nav2_costmap_2d::Layer)

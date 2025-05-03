#include "map_cost_extend/behaviors/back_up_free_space.hpp"

namespace pb_nav2_behaviors
{

// 定义常量
constexpr float DEFAULT_MAX_RADIUS = 1.0;
constexpr float DEFAULT_ANGLE_INCREMENT = M_PI / 32.0;
constexpr int DEFAULT_SERVICE_TIMEOUT = 5;  // seconds
constexpr float OCCUPIED_CELL_COST = 253.0;

// 配置行为参数
void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"无法获取ROS节点"};
  }

  // 声明并获取参数，使用更安全的参数处理
  try {
    nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
    nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(DEFAULT_MAX_RADIUS));
    nav2_util::declare_parameter_if_not_declared(
      node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
    nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("max_radius", max_radius_);
    node->get_parameter("service_name", service_name_);
    node->get_parameter("visualize", visualize_);

    // 参数验证
    if (max_radius_ <= 0.0) {
      RCLCPP_WARN(logger_, "无效的max_radius参数值 (%f)，使用默认值 %f", max_radius_, DEFAULT_MAX_RADIUS);
      max_radius_ = DEFAULT_MAX_RADIUS;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "参数配置失败: %s", e.what());
    throw;
  }

  // 创建服务客户端
  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", rclcpp::QoS(1).transient_local());
    marker_pub_->on_activate();
  }
}

// 清理资源
void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

// 执行行为的初始化
nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (!command) {
    RCLCPP_ERROR(logger_, "收到空的命令指针");
    return nav2_behaviors::Status::FAILED;
  }

  // 等待代价地图服务
  if (!waitForCostmapService()) {
    return nav2_behaviors::Status::FAILED;
  }

  // 获取代价地图
  auto costmap_response = getCostmap();
  if (!costmap_response) {
    return nav2_behaviors::Status::FAILED;
  }

  // 获取初始位姿
  if (!updateRobotPose(initial_pose_)) {
    return nav2_behaviors::Status::FAILED;
  }

  // 计算最佳后退方向
  geometry_msgs::msg::Pose2D current_pose = convertToPose2D(initial_pose_);
  float best_angle = findBestDirection(costmap_response->map, current_pose, -M_PI, M_PI, 
                                     max_radius_, DEFAULT_ANGLE_INCREMENT);

  // 设置运动参数
  setMotionParameters(command, best_angle);

  RCLCPP_INFO(logger_, "开始后退，距离: %.2f 米, 角度: %.2f 弧度", command_x_, best_angle);
  return nav2_behaviors::Status::SUCCEEDED;
}

// 循环更新行为
nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  // 检查时间是否超出允许范围
  if ((end_time_ - clock_->now()).seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(logger_, "超过时间限制，退出行为");
    return nav2_behaviors::Status::FAILED;
  }

  // 获取机器人当前位姿
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "无法获取机器人当前位姿");
    return nav2_behaviors::Status::FAILED;
  }

  // 计算移动距离
  float distance = hypot(
    initial_pose_.pose.position.x - current_pose.pose.position.x,
    initial_pose_.pose.position.y - current_pose.pose.position.y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  // 检查是否达到目标距离
  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // 发布速度指令
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = twist_x_;
  cmd_vel->linear.y = twist_y_;

  // 检查是否存在碰撞
  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(distance, cmd_vel.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "检测到碰撞，退出行为");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

// 找到最佳后退方向
float BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  // 使用局部变量缓存常用值，减少重复计算
  const float resolution = costmap.metadata.resolution;
  const float origin_x = costmap.metadata.origin.position.x;
  const float origin_y = costmap.metadata.origin.position.y;
  const int size_x = costmap.metadata.size_x;
  const int size_y = costmap.metadata.size_y;
  const unsigned char* costmap_data = costmap.data.data();

  // 预计算地图边界
  const float map_min_x = origin_x;
  const float map_max_x = origin_x + (size_x * resolution);
  const float map_min_y = origin_y;
  const float map_max_y = origin_y + (size_y * resolution);

  // 优化角度搜索
  std::vector<std::pair<float, bool>> angle_safety;
  angle_safety.reserve(static_cast<size_t>((end_angle - start_angle) / angle_increment) + 1);

  // 使用标准库算法代替OpenMP
  const int num_angles = static_cast<int>((end_angle - start_angle) / angle_increment) + 1;
  std::vector<float> cos_values(num_angles);
  std::vector<float> sin_values(num_angles);
  for (int i = 0; i < num_angles; ++i) {
    const float angle = start_angle + i * angle_increment;
    cos_values[i] = std::cos(angle);
    sin_values[i] = std::sin(angle);
  }

  float best_angle = start_angle;
  float final_safe_angle = 0.0f;
  float final_unsafe_angle = 0.0f;

  float first_safe_angle = -1.0f;
  float last_unsafe_angle = -1.0f;

  for (int angle_index = 0; angle_index < num_angles; ++angle_index) {
    float angle = start_angle + angle_index * angle_increment;
    bool is_safe = true;

    for (float r = 0; r <= radius; r += resolution) {
      float x = pose.x + r * cos_values[angle_index];
      float y = pose.y + r * sin_values[angle_index];

      if (x >= map_min_x && x <= map_max_x && y >= map_min_y && y <= map_max_y) {
        int i = static_cast<int>((x - origin_x) / resolution);
        int j = static_cast<int>((y - origin_y) / resolution);

        if (i >= 0 && i < size_x && j >= 0 && j < size_y) {
          if (costmap_data[i + j * size_x] >= OCCUPIED_CELL_COST) {
            is_safe = false;
            break;
          }
        } else {
          is_safe = false;
          break;
        }
      } else {
        is_safe = false;
        break;
      }
    }

    if (is_safe) {
      if (first_safe_angle == -1.0f) {
        first_safe_angle = angle;
      }
    } else {
      if (first_safe_angle != -1.0f && last_unsafe_angle == -1.0f) {
        last_unsafe_angle = angle;
      }
    }

    if (first_safe_angle != -1.0f && last_unsafe_angle != -1.0f &&
        (last_unsafe_angle - first_safe_angle > final_unsafe_angle - final_safe_angle)) {
      final_safe_angle = first_safe_angle;
      final_unsafe_angle = last_unsafe_angle;
      first_safe_angle = -1.0f;
      last_unsafe_angle = -1.0f;
    }
  }

  if (final_safe_angle == 0.0f && final_unsafe_angle == 0.0f) {
    best_angle = start_angle;
  } else {
    best_angle = (final_safe_angle + final_unsafe_angle) / 2.0f;
  }

  if (visualize_) {
    visualize(pose, radius, final_safe_angle, final_unsafe_angle);
  }

  return best_angle;
}

// 优化碰撞检测函数
bool BackUpFreeSpace::isCollisionFree(float distance, geometry_msgs::msg::Twist* cmd_vel, 
                                    const geometry_msgs::msg::Pose2D& pose)
{
  // 获取局部代价地图进行快速检查
  auto costmap_response = getCostmap();
  if (!costmap_response) {
    return false;
  }

  // 使用预测轨迹进行碰撞检测
  const float prediction_time = 1.0;  // 预测1秒内的轨迹
  const float time_step = 0.1;       // 每0.1秒检查一次
  
  for (float t = 0; t < prediction_time; t += time_step) {
    // 预测位置
    float pred_x = pose.x + cmd_vel->linear.x * t;
    float pred_y = pose.y + cmd_vel->linear.y * t;
    
    // 检查预测位置是否安全，使用response中的map字段
    if (!isPointSafe(costmap_response->map, pred_x, pred_y)) {
      return false;
    }
  }
  
  return true;
}

// 检查单个点是否安全的辅助函数
bool BackUpFreeSpace::isPointSafe(const nav2_msgs::msg::Costmap& costmap, float x, float y)
{
  const float resolution = costmap.metadata.resolution;
  const float origin_x = costmap.metadata.origin.position.x;
  const float origin_y = costmap.metadata.origin.position.y;
  const int size_x = costmap.metadata.size_x;
  const int size_y = costmap.metadata.size_y;
  
  // 转换为栅格坐标
  int grid_x = static_cast<int>((x - origin_x) / resolution);
  int grid_y = static_cast<int>((y - origin_y) / resolution);
  
  // 检查是否在地图范围内
  if (grid_x < 0 || grid_x >= size_x || grid_y < 0 || grid_y >= size_y) {
    return false;
  }
  
  // 检查代价值
  return costmap.data[grid_y * size_x + grid_x] < OCCUPIED_CELL_COST;
}

// 收集自由空间点
std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

// 可视化自由空间
void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

// 新增辅助方法
bool BackUpFreeSpace::waitForCostmapService()
{
  if (!costmap_client_->wait_for_service(std::chrono::seconds(DEFAULT_SERVICE_TIMEOUT))) {
    RCLCPP_ERROR(logger_, "代价地图服务不可用");
    return false;
  }
  return true;
}

std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> BackUpFreeSpace::getCostmap()
{
  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto future = costmap_client_->async_send_request(request);
  
  if (future.wait_for(std::chrono::seconds(DEFAULT_SERVICE_TIMEOUT)) != std::future_status::ready) {
    RCLCPP_ERROR(logger_, "获取代价地图超时");
    return nullptr;
  }
  return future.get();
}

bool BackUpFreeSpace::updateRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  if (!nav2_util::getCurrentPose(pose, *tf_, global_frame_, 
                               robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "无法获取机器人位姿");
    return false;
  }
  return true;
}

geometry_msgs::msg::Pose2D BackUpFreeSpace::convertToPose2D(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.pose.position.x;
  pose2d.y = pose.pose.position.y;
  pose2d.theta = tf2::getYaw(pose.pose.orientation);
  return pose2d;
}

void BackUpFreeSpace::setMotionParameters(const std::shared_ptr<const BackUpAction::Goal> command, float angle)
{
  twist_x_ = std::cos(angle) * command->speed;
  twist_y_ = std::sin(angle) * command->speed;
  command_x_ = command->target.x;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)

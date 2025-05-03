/**
 * @file path_tracking_controller.cpp
 * @brief 路径跟踪控制器的实现文件
 * @details 该控制器实现了基于 PID 控制和纯追踪算法的路径跟踪功能
 */

#include "path_tracking_controller/path_tracking_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d; 
using rcl_interfaces::msg::ParameterType;

namespace path_tracking_controller
{

/**
 * @brief 配置控制器参数
 * @param parent 父节点指针
 * @param name 控制器名称
 * @param tf TF 变换缓冲区
 * @param costmap_ros 代价地图 ROS 接口
 */
void PathTrackingController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // 锁定父节点，确保生命周期节点的安全访问
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  // 初始化成员变量
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 默认参数值
  constexpr double DEFAULT_TRANSFORM_TOLERANCE = 1.0;
  constexpr double DEFAULT_CONTROL_FREQUENCY = 20.0;
  
  double transform_tolerance = DEFAULT_TRANSFORM_TOLERANCE;
  double control_frequency = DEFAULT_CONTROL_FREQUENCY;
  max_robot_pose_search_dist_ = getCostmapMaxExtent();

  // 定义默认参数列表
  const std::vector<std::pair<std::string, rclcpp::ParameterValue>> default_params = {
    {".translation_kp", rclcpp::ParameterValue(3.0)},  // 平移PID比例系数
    {".translation_ki", rclcpp::ParameterValue(0.1)},  // 平移PID积分系数
    {".translation_kd", rclcpp::ParameterValue(0.3)},  // 平移PID微分系数
    {".enable_rotation", rclcpp::ParameterValue(true)},  // 是否启用旋转控制
    {".rotation_kp", rclcpp::ParameterValue(3.0)},  // 旋转PID比例系数
    {".rotation_ki", rclcpp::ParameterValue(0.1)},  // 旋转PID积分系数
    {".rotation_kd", rclcpp::ParameterValue(0.3)},  // 旋转PID微分系数
    {".transform_tolerance", rclcpp::ParameterValue(0.1)},  // 变换容差
    {".min_max_sum_error", rclcpp::ParameterValue(1.0)},  // 最小最大误差和
    {".lookahead_dist", rclcpp::ParameterValue(0.3)},  // 前瞻距离
    {".use_velocity_scaled_lookahead_dist", rclcpp::ParameterValue(true)},  // 是否使用速度缩放的前瞻距离
    {".min_lookahead_dist", rclcpp::ParameterValue(0.2)},  // 最小前瞻距离
    {".max_lookahead_dist", rclcpp::ParameterValue(1.0)},  // 最大前瞻距离
    {".lookahead_time", rclcpp::ParameterValue(1.0)},  // 前瞻时间
    {".use_interpolation", rclcpp::ParameterValue(true)},  // 是否使用插值
    {".use_rotate_to_heading", rclcpp::ParameterValue(true)},  // 是否使用朝向旋转
    {".use_rotate_to_heading_treshold", rclcpp::ParameterValue(0.1)},  // 朝向旋转阈值
    {".min_approach_linear_velocity", rclcpp::ParameterValue(0.05)},  // 最小接近线速度
    {".approach_velocity_scaling_dist", rclcpp::ParameterValue(0.6)},  // 接近速度缩放距离
    {".v_linear_min", rclcpp::ParameterValue(-3.0)},  // 最小线速度
    {".v_linear_max", rclcpp::ParameterValue(3.0)},  // 最大线速度
    {".v_angular_min", rclcpp::ParameterValue(-3.0)},  // 最小角速度
    {".v_angular_max", rclcpp::ParameterValue(3.0)},  // 最大角速度
    {".max_robot_pose_search_dist", rclcpp::ParameterValue(getCostmapMaxExtent())},  // 最大机器人位姿搜索距离
    {".curvature_min", rclcpp::ParameterValue(0.4)},  // 最小曲率
    {".curvature_max", rclcpp::ParameterValue(0.7)},  // 最大曲率
    {".reduction_ratio_at_high_curvature", rclcpp::ParameterValue(0.5)},  // 高曲率时的减速比例
    {".curvature_forward_dist", rclcpp::ParameterValue(0.7)},  // 曲率前向距离
    {".curvature_backward_dist", rclcpp::ParameterValue(0.3)},  // 曲率后向距离
    {".max_velocity_scaling_factor_rate", rclcpp::ParameterValue(0.9)}  // 最大速度缩放因子率
  };

  // 声明所有参数
  for (const auto& param : default_params) {
    declare_parameter_if_not_declared(node, plugin_name_ + param.first, param.second);
  }

  // 获取所有参数值
  node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
  node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
  node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
  node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
  node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
  node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
  node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading_treshold", use_rotate_to_heading_treshold_);
  node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
  node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
  node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
  node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
  node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
  node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".curvature_min", curvature_min_);
  node->get_parameter(plugin_name_ + ".curvature_max", curvature_max_);
  node->get_parameter(plugin_name_ + ".reduction_ratio_at_high_curvature", reduction_ratio_at_high_curvature_);
  node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
  node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
  node->get_parameter(plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);

  node->get_parameter("controller_frequency", control_frequency);

  // 设置变换容差和控制周期
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  // 设置QoS参数
  rclcpp::QoS qos(10);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  // 创建发布者
  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", qos);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", qos);
  curvature_points_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("curvature_points_marker_array", qos);
  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // 初始化PID控制器
  move_pid_ = std::make_shared<PID>(
    control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
    translation_ki_);
  heading_pid_ = std::make_shared<PID>(
    control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_, rotation_ki_);
}

/**
 * @brief 清理控制器资源
 */
void PathTrackingController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " path_tracking_controller::PathTrackingController",
    plugin_name_.c_str());
  local_path_pub_.reset();
  carrot_pub_.reset();
  curvature_points_pub_.reset();
  cmd_vel_pub_.reset();
}

/**
 * @brief 激活控制器
 */
void PathTrackingController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::PathTrackingController",
    plugin_name_.c_str());
  local_path_pub_->on_activate();
  carrot_pub_->on_activate();
  curvature_points_pub_->on_activate();
}

/**
 * @brief 停用控制器
 */
void PathTrackingController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::PathTrackingController",
    plugin_name_.c_str());
  local_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  curvature_points_pub_->on_deactivate();
  dyn_params_handler_.reset();
}

/**
 * @brief 计算速度命令
 * @param pose 当前位姿
 * @param velocity 当前速度
 * @param goal_checker 目标检查器
 * @return 计算得到的速度命令
 */
geometry_msgs::msg::TwistStamped PathTrackingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // 获取代价地图并加锁
  auto costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // 转换全局路径到局部坐标系
  const auto& transformed_plan = transformGlobalPlan(pose);
  // 获取前瞻距离
  double lookahead_dist = getLookAheadDistance(velocity);

  // 获取前瞻点
  const auto& carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  // 计算到前瞻点的距离和角度
  const double carrot_x = carrot_pose.pose.position.x;
  const double carrot_y = carrot_pose.pose.position.y;
  const double lin_dist = std::hypot(carrot_x, carrot_y);
  const double theta_dist = std::atan2(carrot_y, carrot_x);
  double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);

  // 如果启用了朝向旋转，则使用路径终点的朝向
  if (use_rotate_to_heading_) {
    angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    if (std::abs(angle_to_goal) > use_rotate_to_heading_treshold_) {
      return createZeroVelocityCommand(pose.header);
    }
  }

  // 计算线速度和角速度
  double lin_vel = move_pid_->calculate(lin_dist, 0);
  double angular_vel = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0) : 0.0;

  // 应用曲率限制和接近速度缩放
  applyCurvatureLimitation(transformed_plan, carrot_pose, lin_vel);
  applyApproachVelocityScaling(transformed_plan, lin_vel);

  // 检查碰撞并处理
  if (isCollisionDetected(transformed_plan)) {
    // 尝试碰撞恢复
    if (attemptCollisionRecovery(pose, velocity, transformed_plan)) {
      // 恢复成功后重新计算速度命令
      return computeVelocityCommands(pose, velocity, nullptr);
    }

    // 恢复失败，返回零速度命令
    return createZeroVelocityCommand(pose.header);
  }

  // 创建速度命令
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = lin_vel * std::cos(theta_dist);
  cmd_vel.twist.linear.y = lin_vel * std::sin(theta_dist);
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}

/**
 * @brief 检测到碰撞后尝试相反方向移动并动态检测是否脱离碰撞代价
 * @param pose 当前位姿
 * @param velocity 当前速度
 * @param transformed_plan 转换后的路径
 * @return 是否成功脱离碰撞
 */
bool PathTrackingController::attemptCollisionRecovery(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  const nav_msgs::msg::Path & transformed_plan)
{
  RCLCPP_WARN(logger_, "Collision detected! Attempting recovery...");

  // 设置相反方向的速度命令
  geometry_msgs::msg::TwistStamped recovery_cmd;
  recovery_cmd.header = pose.header;
  recovery_cmd.twist.linear.x = -velocity.linear.x * 0.5;  // 反向移动，速度减半
  recovery_cmd.twist.linear.y = -velocity.linear.y * 0.5;
  recovery_cmd.twist.angular.z = 0.0;

  // 发布速度命令并动态检测是否脱离碰撞
  rclcpp::Rate rate(10);  // 10 Hz
  for (int i = 0; i < 50; ++i) {  // 尝试 5 秒内脱离碰撞
    cmd_vel_pub_->publish(recovery_cmd.twist);
    rclcpp::spin_some(node_.lock()->get_node_base_interface());

    // 检查是否脱离碰撞
    if (!isCollisionDetected(transformed_plan)) {
      RCLCPP_INFO(logger_, "Successfully recovered from collision.");
      return true;
    }

    rate.sleep();
  }

  RCLCPP_ERROR(logger_, "Failed to recover from collision.");
  return false;
}

/**
 * @brief 设置全局路径
 * @param path 全局路径
 */
void PathTrackingController::setPlan(const nav_msgs::msg::Path & path) { global_plan_ = path; }

/**
 * @brief 设置速度限制
 * @param speed_limit 速度限制
 * @param percentage 是否为百分比
 */
void PathTrackingController::setSpeedLimit(
  const double & /*速度限制*/, const bool & /*百分比*/)
{
  RCLCPP_WARN(logger_, "Speed limit is not implemented in this controller.");
}

/**
 * @brief 转换全局路径到局部坐标系
 * @param pose 当前位姿
 * @return 转换后的路径
 */
nav_msgs::msg::Path PathTrackingController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // 转换机器人位姿到全局路径坐标系
  const auto& robot_pose = pose;
  geometry_msgs::msg::PoseStamped transformed_robot_pose;
  if (!transformPose(global_plan_.header.frame_id, robot_pose, transformed_robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // 获取代价地图最大范围
  const double max_costmap_extent = getCostmapMaxExtent();
  // 找到距离机器人最近的路径点
  const auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // 找到最近的路径点
  const auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&transformed_robot_pose](const auto & ps) {
      return nav2_util::geometry_utils::euclidean_distance(transformed_robot_pose, ps);
    });

  // 找到超出代价地图范围的路径点
  const auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&transformed_robot_pose, max_costmap_extent](const auto & ps) {
      return nav2_util::geometry_utils::euclidean_distance(ps, transformed_robot_pose) > max_costmap_extent;
    });

  // 转换路径点到局部坐标系
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.poses.reserve(std::distance(transformation_begin, transformation_end));

  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    [this, &transformed_robot_pose](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = transformed_robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    });

  // 设置转换后路径的帧ID和时间戳
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = transformed_robot_pose.header.stamp;

  // 更新全局路径
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  local_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

/**
 * @brief 创建前瞻点消息
 * @param carrot_pose 前瞻点位姿
 * @return 前瞻点消息
 */
std::unique_ptr<geometry_msgs::msg::PointStamped> PathTrackingController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;
  return carrot_msg;
}

/**
 * @brief 获取前瞻点
 * @param lookahead_dist 前瞻距离
 * @param transformed_plan 转换后的路径
 * @return 前瞻点位姿
 */
geometry_msgs::msg::PoseStamped PathTrackingController::getLookAheadPoint(
  const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan)
{
  // 找到距离大于前瞻距离的第一个路径点
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // 如果找不到，则使用最后一个路径点
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // 如果启用了插值，则在前一个路径点和当前路径点之间进行插值
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
    
    // 计算插值点的朝向
    double prev_yaw = tf2::getYaw(prev_pose_it->pose.orientation);
    double curr_yaw = tf2::getYaw(goal_pose_it->pose.orientation);
    double ratio = (lookahead_dist - hypot(prev_pose_it->pose.position.x, prev_pose_it->pose.position.y)) /
                  (hypot(goal_pose_it->pose.position.x, goal_pose_it->pose.position.y) -
                   hypot(prev_pose_it->pose.position.x, prev_pose_it->pose.position.y));
    
    // 角度插值（考虑角度跨越2π的情况）
    double angle_diff = curr_yaw - prev_yaw;
    if (angle_diff > M_PI) {
      angle_diff -= 2 * M_PI;
    } else if (angle_diff < -M_PI) {
      angle_diff += 2 * M_PI;
    }
    double interpolated_yaw = prev_yaw + ratio * angle_diff;
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), interpolated_yaw));
    return pose;
  }

  return *goal_pose_it;
}

/**
 * @brief 计算圆与线段的交点
 * @param p1 线段起点
 * @param p2 线段终点
 * @param r 圆半径
 * @return 交点坐标
 */
geometry_msgs::msg::Point PathTrackingController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r)
{
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double d = x1 * y2 - x2 * y1;

  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - d * d);
  p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

/**
 * @brief 获取代价地图最大范围
 * @return 最大范围
 */
double PathTrackingController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

/**
 * @brief 转换位姿到指定坐标系
 * @param frame 目标坐标系
 * @param in_pose 输入位姿
 * @param out_pose 输出位姿
 * @return 是否转换成功
 */
bool PathTrackingController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

/**
 * @brief 检查是否检测到碰撞
 * @param path 路径
 * @return 是否检测到碰撞
 */
bool PathTrackingController::isCollisionDetected(const nav_msgs::msg::Path & path)
{
  const auto costmap = costmap_ros_->getCostmap();
  const auto& poses = path.poses;
  
  return std::any_of(poses.begin(), poses.end(), [costmap](const auto & pose_stamped) {
    const auto & pose = pose_stamped.pose;
    unsigned int mx, my;
    if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
      return costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    return false;
  });
}

/**
 * @brief 获取前瞻距离
 * @param speed 当前速度
 * @return 前瞻距离
 */
double PathTrackingController::getLookAheadDistance(const geometry_msgs::msg::Twist & speed)
{
  double lookahead_dist = lookahead_dist_;

  // 如果启用了速度缩放的前瞻距离，则根据速度计算
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  return lookahead_dist;
}

/**
 * @brief 计算接近速度缩放因子
 * @param transformed_path 转换后的路径
 * @return 缩放因子
 */
double PathTrackingController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path) const
{
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

/**
 * @brief 应用接近速度缩放
 * @param path 路径
 * @param linear_vel 线速度
 */
void PathTrackingController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  linear_vel = std::min(linear_vel, approach_vel);
}

/**
 * @brief 应用曲率限制
 * @param path 路径
 * @param lookahead_pose 前瞻点位姿
 * @param linear_vel 线速度
 */
void PathTrackingController::applyCurvatureLimitation(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double & linear_vel)
{
  // 计算曲率
  double curvature =
    calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);

  double scaled_linear_vel = linear_vel;
  if (curvature > curvature_min_) {
    double reduction_ratio = 1.0;
    if (curvature > curvature_max_) {
      reduction_ratio = reduction_ratio_at_high_curvature_;
    } else {
      reduction_ratio = 1.0 - (curvature - curvature_min_) / (curvature_max_ - curvature_min_) *
                                (1.0 - reduction_ratio_at_high_curvature_);
    }

    double target_scaled_vel = linear_vel * reduction_ratio;
    scaled_linear_vel =
      last_velocity_scaling_factor_ + std::clamp(
                                        target_scaled_vel - last_velocity_scaling_factor_,
                                        -max_velocity_scaling_factor_rate_ * control_duration_,
                                        max_velocity_scaling_factor_rate_ * control_duration_);
  }
  scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);

  linear_vel = std::min(linear_vel, scaled_linear_vel);
  last_velocity_scaling_factor_ = linear_vel;
}

/**
 * @brief 计算曲率
 * @param path 路径
 * @param lookahead_pose 前瞻点位姿
 * @param forward_dist 前向距离
 * @param backward_dist 后向距离
 * @return 曲率值
 */
double PathTrackingController::calculateCurvature(
  const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
  double forward_dist, double backward_dist) const
{
  geometry_msgs::msg::PoseStamped backward_pose, forward_pose;
  std::vector<double> cumulative_distances = calculateCumulativeDistances(path);

  double lookahead_pose_cumulative_distance = 0.0;
  geometry_msgs::msg::PoseStamped robot_base_frame_pose;
  robot_base_frame_pose.pose = geometry_msgs::msg::Pose();
  lookahead_pose_cumulative_distance =
    nav2_util::geometry_utils::euclidean_distance(robot_base_frame_pose, lookahead_pose);

  // 找到前向和后向点
  backward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance - backward_dist);

  forward_pose = findPoseAtDistance(
    path, cumulative_distances, lookahead_pose_cumulative_distance + forward_dist);

  // 计算曲率半径
  double curvature_radius = calculateCurvatureRadius(
    backward_pose.pose.position, lookahead_pose.pose.position, forward_pose.pose.position);
  
  // 添加曲率平滑处理
  double curvature = 1.0 / curvature_radius;
  if (std::isnan(curvature) || std::isinf(curvature)) {
    curvature = 0.0;
  }
  
  // 使用移动平均平滑曲率值
  static constexpr size_t CURVATURE_WINDOW_SIZE = 5;
  static std::deque<double> curvature_history;
  curvature_history.push_back(curvature);
  if (curvature_history.size() > CURVATURE_WINDOW_SIZE) {
    curvature_history.pop_front();
  }
  
  double smoothed_curvature = 0.0;
  for (const auto & c : curvature_history) {
    smoothed_curvature += c;
  }
  smoothed_curvature /= curvature_history.size();
  
  visualizeCurvaturePoints(backward_pose, forward_pose);
  return smoothed_curvature;
}

/**
 * @brief 计算曲率半径
 * @param near_point 近点
 * @param current_point 当前点
 * @param far_point 远点
 * @return 曲率半径
 */
double PathTrackingController::calculateCurvatureRadius(
  const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
  const geometry_msgs::msg::Point & far_point) const
{
  double x1 = near_point.x, y1 = near_point.y;
  double x2 = current_point.x, y2 = current_point.y;
  double x3 = far_point.x, y3 = far_point.y;

  // 计算圆心坐标
  double center_x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) +
                     (x3 * x3 + y3 * y3) * (y1 - y2)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double center_y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) +
                     (x3 * x3 + y3 * y3) * (x2 - x1)) /
                    (2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)));
  double radius = std::hypot(x2 - center_x, y2 - center_y);
  if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
    return 1e9;
  }
  return radius;
}

/**
 * @brief 可视化曲率点
 * @param backward_pose 后向点位姿
 * @param forward_pose 前向点位姿
 */
void PathTrackingController::visualizeCurvaturePoints(
  const geometry_msgs::msg::PoseStamped & backward_pose,
  const geometry_msgs::msg::PoseStamped & forward_pose) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 创建后向点标记
  visualization_msgs::msg::Marker near_marker;
  near_marker.header = backward_pose.header;
  near_marker.ns = "curvature_points";
  near_marker.id = 0;
  near_marker.type = visualization_msgs::msg::Marker::SPHERE;
  near_marker.action = visualization_msgs::msg::Marker::ADD;
  near_marker.pose = backward_pose.pose;
  near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
  near_marker.color.g = 1.0;
  near_marker.color.a = 1.0;

  // 创建前向点标记
  visualization_msgs::msg::Marker far_marker;
  far_marker.header = forward_pose.header;
  far_marker.ns = "curvature_points";
  far_marker.id = 1;
  far_marker.type = visualization_msgs::msg::Marker::SPHERE;
  far_marker.action = visualization_msgs::msg::Marker::ADD;
  far_marker.pose = forward_pose.pose;
  far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
  far_marker.color.r = 1.0;
  far_marker.color.a = 1.0;

  marker_array.markers.push_back(near_marker);
  marker_array.markers.push_back(far_marker);

  curvature_points_pub_->publish(marker_array);
}

/**
 * @brief 计算累积距离
 * @param path 路径
 * @return 累积距离数组
 */
std::vector<double> PathTrackingController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev_pose = path.poses[i - 1].pose.position;
    const auto & curr_pose = path.poses[i].pose.position;
    double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
    cumulative_distances.push_back(cumulative_distances.back() + distance);
  }
  return cumulative_distances;
}

/**
 * @brief 在指定距离处找到位姿
 * @param path 路径
 * @param cumulative_distances 累积距离数组
 * @param target_distance 目标距离
 * @return 找到的位姿
 */
geometry_msgs::msg::PoseStamped PathTrackingController::findPoseAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.empty() || cumulative_distances.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }
  if (target_distance <= 0.0) {
    return path.poses.front();
  }
  if (target_distance >= cumulative_distances.back()) {
    return path.poses.back();
  }
  auto it =
    std::lower_bound(cumulative_distances.begin(), cumulative_distances.end(), target_distance);
  size_t index = std::distance(cumulative_distances.begin(), it);

  if (index == 0) {
    return path.poses.front();
  }

  // 在相邻路径点之间进行插值
  double ratio = (target_distance - cumulative_distances[index - 1]) /
                 (cumulative_distances[index] - cumulative_distances[index - 1]);
  geometry_msgs::msg::PoseStamped pose1 = path.poses[index - 1];
  geometry_msgs::msg::PoseStamped pose2 = path.poses[index];

  geometry_msgs::msg::PoseStamped interpolated_pose;
  interpolated_pose.header = pose2.header;
  interpolated_pose.pose.position.x =
    pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
  interpolated_pose.pose.position.y =
    pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
  interpolated_pose.pose.position.z =
    pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
  interpolated_pose.pose.orientation = pose2.pose.orientation;

  return interpolated_pose;
}

/**
 * @brief 计算碰撞避让方向
 * @param path 路径
 * @return 避让方向角度（弧度）
 */
double PathTrackingController::calculateCollisionAvoidanceDirection(const nav_msgs::msg::Path & path) const
{
  const auto costmap = costmap_ros_->getCostmap();
  double total_x = 0.0;
  double total_y = 0.0;
  int count = 0;

  // 计算所有碰撞点的平均位置
  for (const auto & pose_stamped : path.poses) {
    const auto & pose = pose_stamped.pose;
    unsigned int mx, my;
    if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        total_x += pose.position.x;
        total_y += pose.position.y;
        count++;
      }
    }
  }

  if (count == 0) {
    return 0.0;
  }

  // 计算碰撞点的中心位置
  double center_x = total_x / count;
  double center_y = total_y / count;

  // 计算避让方向（与碰撞点相反的方向）
  double avoidance_angle = std::atan2(-center_y, -center_x);
  return avoidance_angle;
}

/**
 * @brief 生成避让速度命令
 * @param header 消息头
 * @param avoidance_angle 避让方向角度
 * @return 避让速度命令
 */
geometry_msgs::msg::TwistStamped PathTrackingController::createAvoidanceCommand(
  const std_msgs::msg::Header & header, double avoidance_angle) const
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = header;
  
  // 设置避让速度
  double avoidance_speed = min_approach_linear_velocity_ * 0.5;  // 使用较小的速度进行避让
  cmd_vel.twist.linear.x = avoidance_speed * std::cos(avoidance_angle);
  cmd_vel.twist.linear.y = avoidance_speed * std::sin(avoidance_angle);
  
  // 添加一些旋转以帮助避让
  cmd_vel.twist.angular.z = avoidance_angle * 0.5;
  
  return cmd_vel;
}

/**
 * @brief 动态参数回调函数
 * @param parameters 参数列表
 * @return 参数设置结果
 */
rcl_interfaces::msg::SetParametersResult PathTrackingController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".translation_kp") {
        translation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_ki") {
        translation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".translation_kd") {
        translation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kp") {
        rotation_kp_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_ki") {
        rotation_ki_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotation_kd") {
        rotation_kd_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".min_max_sum_error") {
        min_max_sum_error_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".use_rotate_to_heading_treshold") {
        use_rotate_to_heading_treshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_max") {
        v_linear_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_linear_min") {
        v_linear_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_max") {
        v_angular_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".v_angular_min") {
        v_angular_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_min") {
        curvature_min_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_max") {
        curvature_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".reduction_ratio_at_high_curvature") {
        reduction_ratio_at_high_curvature_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_forward_dist") {
        curvature_forward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_backward_dist") {
        curvature_backward_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
        max_velocity_scaling_factor_rate_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_interpolation") {
        use_interpolation_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        use_rotate_to_heading_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

/**
 * @brief 创建零速度命令
 * @param header 消息头
 * @return 零速度命令
 */
geometry_msgs::msg::TwistStamped PathTrackingController::createZeroVelocityCommand(
  const std_msgs::msg::Header & header) const
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = header;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}

};  // 命名空间 path_tracking_controller

// 将此控制器注册为 nav2_core 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  path_tracking_controller::PathTrackingController, nav2_core::Controller)

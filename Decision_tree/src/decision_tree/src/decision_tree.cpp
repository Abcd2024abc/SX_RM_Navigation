#include "decision_tree/decision_tree.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace decision_tree
{
DecisionTree::DecisionTree(const rclcpp::NodeOptions& options) : Node("decision_tree", options)
{
  RCLCPP_INFO(this->get_logger(), "导航系统节点创建");

  try
  {
    initialize();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "初始化失败: %s", e.what());
    throw;
  }
}

DecisionTree::~DecisionTree()
{
  cancel_navigation();
  RCLCPP_INFO(this->get_logger(), "导航系统节点销毁");
}

void DecisionTree::initialize()
{
  // 初始化TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 声明和加载参数
  auto& p = params_;
  auto& pd = params_.decision_params;

  set_params(this, "robot_base_frame", p.robot_base_frame, std::string("base_footprint"));
  set_params(this, "map_topic", p.map_topic, std::string("/map"));
  set_params(this, "position_tolerance", p.position_tolerance, 0.3);
  set_params(this, "navigation_timeout", p.navigation_timeout, 60.0);
  set_params(this, "connection_timeout", p.connection_timeout, 5.0);
  set_params(this, "max_connection_retries", p.max_connection_retries, 3);

  // 决策参数
  set_params(this, "low_hp_threshold", pd.low_hp_threshold, 0.3);        // 30%
  set_params(this, "safe_hp_threshold", pd.safe_hp_threshold, 0.9);      // 90%
  set_params(this, "normal_hp_threshold", pd.normal_hp_threshold, 0.5);  // 50%
  set_params(this, "no_attack_duration", pd.no_attack_duration, 5.0);   // 5秒

  // 位置参数
  set_params(this, "base_position_x", pd.base_x, 0.0);
  set_params(this, "base_position_y", pd.base_y, 0.0);
  set_params(this, "center_position_x", pd.center_x, 0.0);
  set_params(this, "center_position_y", pd.center_y, 0.0);

  // 控制参数
  set_params(this, "activate_search", pd.activate_search, float(5.0));

  logger("参数加载完成:", "info");
  logger("  机器人基础坐标系: " + params_.robot_base_frame, "info");
  logger("  地图话题: " + params_.map_topic, "info");
  logger("  位置容差: " + std::to_string(params_.position_tolerance) + "m", "info");
  logger("  基地位置: (" + std::to_string(params_.decision_params.base_x) + ", " +
             std::to_string(params_.decision_params.base_y) + ")",
         "info");
  logger("  增益点位置: (" + std::to_string(params_.decision_params.center_x) + ", " +
             std::to_string(params_.decision_params.center_y) + ")",
         "info");

  control_pub_ = this->create_publisher<connection_layer::msg::ControlSignal>("control_signal", 10);

  // 初始化各个组件
  initialize_navigation();
  initialize_subscriptions();
  initialize_timers();

  // 设置初始状态
  state_.current_state = DecisionState::PAUSED;
  state_.last_state_change_time = get_current_time();

  logger("导航系统初始化完成", "info");
}

void DecisionTree::initialize_subscriptions()
{
  // 地图订阅
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      params_.map_topic, rclcpp::QoS(10).reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_callback(msg); });

  // 游戏状态订阅
  game_status_sub_ = this->create_subscription<connection_layer::msg::GameStatus>(
      "game_status", rclcpp::QoS(10).reliable(),
      [this](const connection_layer::msg::GameStatus::SharedPtr msg) { game_status_callback(msg); });

  // 机器人状态订阅
  robot_status_sub_ = this->create_subscription<connection_layer::msg::RobotStatus>(
      "robot_status", rclcpp::QoS(10).reliable(),
      [this](const connection_layer::msg::RobotStatus::SharedPtr msg) { robot_status_callback(msg); });

  logger("所有订阅器初始化完成", "info");
}

void DecisionTree::initialize_timers()
{
  // 状态更新定时器
  status_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { update_status(); });

  // 决策树定时器
  decision_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { decision_tree_callback(); });

  // 攻击对策定时器
  attack_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { attack_check_callback(); });
}

void DecisionTree::initialize_navigation()
{
  nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

// =============== 回调函数 ===============

void DecisionTree::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  try
  {
    // 存储地图数据
    map_data_ = std::vector<int8_t>(msg->data.begin(), msg->data.end());
    map_metadata_ = msg;
    state_.map_available = true;

    logger("接收到地图数据:", "info");
    logger("  尺寸: " + std::to_string(msg->info.width) + "x" + std::to_string(msg->info.height), "info");
    logger("  原点: (" + std::to_string(msg->info.origin.position.x) + ", " +
               std::to_string(msg->info.origin.position.y) + ")",
           "info");
  }
  catch (const std::exception& e)
  {
    logger("处理地图数据失败: " + std::string(e.what()), "error");
    state_.map_available = false;
  }
}

void DecisionTree::game_status_callback(const connection_layer::msg::GameStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);

  // 更新游戏状态
  game_state_.game_progress = static_cast<int>(msg->progress);
  game_state_.stage_remain_time = msg->remaining;

  game_state_.center_gain_point = static_cast<int>(msg->gain);

  // 根据比赛进程调整决策状态
  switch (game_state_.game_progress)
  {
    case 0:
      break;

    case 1:
      break;

    case 2:
      break;

    case 3:
      break;

    case 4:
      logger("比赛开始", "info");
      state_.is_active = true;
      break;

    case 5:
      change_state(DecisionState::PAUSED);
      state_.is_active = false;
      cancel_navigation();
      logger("比赛结束，停止所有行动", "info");
      break;
  }

  logger("游戏状态更新 - 进程: " + std::to_string(game_state_.game_progress) +
             ", 剩余时间: " + std::to_string(game_state_.stage_remain_time) + "秒",
         "debug");
}

void DecisionTree::robot_status_callback(const connection_layer::msg::RobotStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);

  if (game_state_.maximum_hp == -1 && static_cast<int>(msg->hp) > 5)
    game_state_.maximum_hp = static_cast<int>(msg->hp);
  // 记录血量变化
  int previous_hp = game_state_.current_hp;
  int current_hp = static_cast<int>(msg->hp);

  if (current_hp < previous_hp)
  {
    // 血量减少，说明受到攻击
    game_state_.last_attack_time = get_current_time();
    game_state_.under_attack = true;
    logger("检测到受到攻击! 当前血量: " + std::to_string(current_hp) + ", 上次血量: " + std::to_string(previous_hp),
           "info");
  }

  // 更新血量
  game_state_.current_hp = current_hp;

  logger("机器人状态更新 - 血量: " + std::to_string(game_state_.current_hp) + "/" +
             std::to_string(game_state_.maximum_hp) + " (" + std::to_string(get_hp_percentage() * 100) + "%)",
         "debug");
}

// =============== 定时器回调 ===============

void DecisionTree::update_status()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // 更新机器人位置
  get_robot_position();

  // 信息
  std::string status_msg;
  status_msg += "状态: " + state_to_string(state_.current_state);
  status_msg += ", 导航中=" + std::to_string(state_.is_navigating);

  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    status_msg += ", 游戏进程=" + std::to_string(game_state_.game_progress);
    status_msg += ", 血量=" + std::to_string(get_hp_percentage() * 100) + "%";
    status_msg += ", 增益点状态=" + std::to_string(game_state_.center_gain_point);
  }

  if (state_.is_navigating)
  {
    double distance = calculate_distance(state_.robot_x, state_.robot_y, state_.target_x, state_.target_y);
    status_msg += ", 目标距离=" + std::to_string(distance) + "m";
  }

  if (state_.robot_y > params_.decision_params.activate_search)
  {
    state_.search = 1.0;
  }
  else
  {
    state_.search = 0.0;
  }
  if (!state_.is_active)
  {
    state_.search = 0.0;
    state_.rotate = 0.0;
  }
  pub_control_signal(state_.rotate, state_.search);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s", status_msg.c_str());
}

void DecisionTree::decision_tree_callback()
{
  if (!state_.is_active)
  {
    return;
  }

  // 更新决策状态
  if (update_decision_state())
  {
    // 执行当前状态
    execute_decision_state();
  };

  state_.last_decision_time = get_current_time();
}

void DecisionTree::attack_check_callback()
{
  if (game_state_.under_attack == false)
    return;

  state_.interrupt_decision = true;
  /// @brief 逻辑待处理

  game_state_.under_attack = false;
  state_.interrupt_decision = false;
}

// =============== 导航方法 ===============

bool DecisionTree::navigate_to_point(double x, double y, double yaw)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // 检查目标点是否在地图边界内
  if (!is_in_map_boundary(x, y))
  {
    logger("目标点不在有效地图边界内", "error");
    return false;
  }

  // 检查当前位置与目标位置的距离
  if (get_robot_position())
  {
    double distance = calculate_distance(state_.robot_x, state_.robot_y, x, y);
    if (distance < params_.position_tolerance)
    {
      logger("已在目标点附近 (距离: " + std::to_string(distance) + "m), 跳过导航", "info");
      return true;
    }
  }

  // 创建目标姿态
  auto goal_pose = geometry_msgs::msg::PoseStamped();
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = this->now();
  goal_pose.pose.position.x = x;
  goal_pose.pose.position.y = y;

  // 设置朝向
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  goal_pose.pose.orientation = tf2::toMsg(quaternion);

  return navigate_to_pose(goal_pose);
}

bool DecisionTree::navigate_to_pose(const geometry_msgs::msg::PoseStamped& pose)
{
  // 取消当前导航
  cancel_navigation();

  // 创建导航目标
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = pose;

  // 设置动作选项
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // 导航发布
  send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr& goal_handle) {
        if (!goal_handle)
        {
          logger("导航目标被拒绝", "error");
          state_.is_navigating = false;
        }
        else
        {
          logger("导航目标已接受", "info");
          current_goal_handle_ = goal_handle;
        }
      };

  // 导航进度
  send_goal_options.feedback_callback = [this](GoalHandleNavigateToPose::SharedPtr,
                                               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "导航进度 - 剩余距离: %.2f米, 导航时间: %d秒",
                         feedback->distance_remaining, feedback->navigation_time.sec);
  };

  // 导航结果
  send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult& result) {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        logger("导航成功完成", "info");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        logger("导航被中止", "warn");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        logger("导航被取消", "info");
        break;
      default:
        logger("未知导航结果", "error");
        break;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.is_navigating = false;
    current_goal_handle_.reset();
  };

  auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);

  state_.is_navigating = true;
  state_.target_x = pose.pose.position.x;
  state_.target_y = pose.pose.position.y;

  logger("开始导航到目标点: (" + std::to_string(pose.pose.position.x) + ", " + std::to_string(pose.pose.position.y) +
             ")",
         "info");

  return true;
}

void DecisionTree::cancel_navigation()
{
  if (current_goal_handle_)
  {
    logger("取消当前导航", "warn");
    nav_action_client_->async_cancel_goal(current_goal_handle_);
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.is_navigating = false;
  state_.target_x = 0.0;
  state_.target_y = 0.0;
  current_goal_handle_.reset();
}

bool DecisionTree::get_robot_position()
{
  try
  {
    // 获取变换
    auto transform = tf_buffer_->lookupTransform("map", params_.robot_base_frame, tf2::TimePointZero);

    // 更新位置
    state_.robot_x = transform.transform.translation.x;
    state_.robot_y = transform.transform.translation.y;

    // 计算偏航角
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    state_.robot_yaw = yaw;

    return true;
  }
  catch (const tf2::TransformException& e)
  {
    logger("获取机器人位置失败: " + std::string(e.what()), "warn");
    return false;
  }
  catch (const std::exception& e)
  {
    logger("获取位置异常: " + std::string(e.what()), "error");
    return false;
  }
}

// =============== 决策树方法 ===============

bool DecisionTree::update_decision_state()
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);
  if (state_.interrupt_decision)
  {
    return false;
  }
  
  // 规则: 当前血量低时返回基地补充
  else if (is_low_hp())
  {
    if (state_.current_state != DecisionState::RETURNING_TO_BASE)
    {
      change_state(DecisionState::RETURNING_TO_BASE);
      return true;
    }
    else
      return false;
  }

  // 规则: 返回基地状态，血量补充到安全继续行动
  else if (state_.current_state == DecisionState::RETURNING_TO_BASE)
  {
    if (is_safe_hp())
    {
      change_state(DecisionState::CAPTURING_CENTER);
      return true;
    }
    else
      return false;
  }

  // 规则：己方未占领则执行占领
  else if (!center_captured())
  {
    if (state_.current_state != DecisionState::CAPTURING_CENTER)
    {
      change_state(DecisionState::CAPTURING_CENTER);
      return true;
    }
    else
      return false;
  }

  // 规则: 如果不在增益点并且己方占领
  else if (!in_key_position(1) && center_captured())
  {
    change_state(DecisionState::APPROACHING_ENEMY);
    return true;
  }

  // 规则: 当血量充足且增益点为己方占领状态，5秒未受到攻击，尝试接近敌方
  else if (is_normal_hp() && center_captured() &&
           has_not_been_attacked_for(params_.decision_params.no_attack_duration) &&
           state_.current_state != DecisionState::APPROACHING_ENEMY)
  {
    change_state(DecisionState::APPROACHING_ENEMY);
    return true;
  }

  // 规则: 如果正在接近敌方基地但条件不再满足，返回抢占中心点
  else if (state_.current_state == DecisionState::APPROACHING_ENEMY && center_captured())
  {
    change_state(DecisionState::CAPTURING_CENTER);
    return true;
  }

  return false;
}

void DecisionTree::execute_decision_state()
{
  switch (state_.current_state)
  {
    case DecisionState::GUARD:
      handle_guard_state();
      break;
    case DecisionState::CAPTURING_CENTER:
      handle_capturing_center_state();
      break;
    case DecisionState::RETURNING_TO_BASE:
      handle_returning_base_state();
      break;
    case DecisionState::APPROACHING_ENEMY:
      handle_approaching_enemy_state();
      break;
    case DecisionState::PAUSED:
      // 暂停状态，不执行任何操作
      break;
  }
}

void DecisionTree::change_state(DecisionState new_state)
{
  if (state_.current_state != new_state)
  {
    state_.previous_state = state_.current_state;
    state_.current_state = new_state;
    state_.last_state_change_time = get_current_time();

    logger("决策状态改变: " + state_to_string(state_.previous_state) + " -> " + state_to_string(state_.current_state),
           "info");

    cancel_navigation();
  }
}

void DecisionTree::handle_guard_state()
{
  state_.rotate = 1.0;
  state_.search = 1.0;
}

void DecisionTree::handle_capturing_center_state()
{
  if (in_key_position(1))
  {
    return;
  }
  else
  {
    logger("抢占增益点", "info");
    navigate_to_point(params_.decision_params.center_x, params_.decision_params.center_y);
  }
}

void DecisionTree::handle_returning_base_state()
{
  if (in_key_position(0))
  {
    return;
  }
  else
  {
    logger("返回基地补充血量", "info");
    navigate_to_point(params_.decision_params.base_x, params_.decision_params.base_y);
  }
}

void DecisionTree::handle_approaching_enemy_state()
{
  double x = params_.decision_params.center_y;
  double y = params_.decision_params.base_x * 0.9;
  logger("压制敌方，目标点: (" + std::to_string(x) + ", " + std::to_string(y) + ")", "info");

  navigate_to_point(x, y);
}

// =============== 决策辅助方法 ===============

bool DecisionTree::in_key_position(const int key) const
{
  if (key == 0)
  {
    double distance = calculate_distance(state_.robot_x, state_.robot_y, params_.decision_params.base_x,
                                         params_.decision_params.base_y);

    if (distance < params_.position_tolerance)
    {
      logger("已在基地", "info");
      return true;
    }
  }
  else if (key == 1)
  {
    double distance = calculate_distance(state_.robot_x, state_.robot_y, params_.decision_params.center_x,
                                         params_.decision_params.center_y);

    if (distance < params_.position_tolerance)
    {
      logger("已在增益点", "info");
      return true;
    }
  }
  return false;
}

bool DecisionTree::center_captured() const
{
  return game_state_.center_gain_point == 1;
}

bool DecisionTree::is_low_hp() const
{
  return get_hp_percentage() < params_.decision_params.low_hp_threshold;
}

bool DecisionTree::is_safe_hp() const
{
  return get_hp_percentage() > params_.decision_params.safe_hp_threshold;
}

bool DecisionTree::is_normal_hp() const
{
  return get_hp_percentage() > params_.decision_params.normal_hp_threshold;
}

bool DecisionTree::has_not_been_attacked_for(double duration) const
{
  double current_time = get_current_time();
  return (current_time - game_state_.last_attack_time) >= duration;
}

double DecisionTree::get_hp_percentage() const
{
  if (game_state_.maximum_hp == 0)
  {
    return 0.0;
  }
  return static_cast<double>(game_state_.current_hp) / static_cast<double>(game_state_.maximum_hp);
}

// =============== 工具函数 ===============

void DecisionTree::pub_control_signal(const float rotate, const float search)
{
  connection_layer::msg::ControlSignal control_msg;
  control_msg.rotate = rotate;
  control_msg.search = search;
  control_pub_->publish(control_msg);
}

double DecisionTree::calculate_distance(double x1, double y1, double x2, double y2) const
{
  double dx = x1 - x2;
  double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

bool DecisionTree::is_in_map_boundary(double x, double y) const
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (!state_.map_available)
  {
    logger("地图数据不可用，无法验证边界", "warn");
    return true;
  }

  const auto& info = map_metadata_->info;
  double min_x = info.origin.position.x;
  double min_y = info.origin.position.y;
  double max_x = min_x + info.width * info.resolution;
  double max_y = min_y + info.height * info.resolution;

  bool x_valid = (min_x <= x) && (x <= max_x);
  bool y_valid = (min_y <= y) && (y <= max_y);

  return x_valid && y_valid;
}

double DecisionTree::get_current_time() const
{
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration<double>(duration).count();
}

void DecisionTree::logger(const std::string& message, const char* type) const
{
  if (std::string(type) == "info")
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  else if (std::string(type) == "warn")
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
  else if (std::string(type) == "error")
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  else if (std::string(type) == "debug")
    RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
}

std::string DecisionTree::state_to_string(DecisionState state) const
{
  switch (state)
  {
    case DecisionState::GUARD:
      return "GUARD";
    case DecisionState::CAPTURING_CENTER:
      return "CAPTURING_CENTER";
    case DecisionState::RETURNING_TO_BASE:
      return "RETURNING_TO_BASE";
    case DecisionState::APPROACHING_ENEMY:
      return "APPROACHING_ENEMY";
    case DecisionState::PAUSED:
      return "PAUSED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace decision_tree

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decision_tree::DecisionTree)
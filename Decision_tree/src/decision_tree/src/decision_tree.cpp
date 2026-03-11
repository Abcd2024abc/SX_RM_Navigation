#include "decision_tree.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace decision
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
  set_parameters();
  calculate_enemy_base_position();

  // 初始化各个组件
  initialize_navigation();
  initialize_subscriptions();
  initialize_timers();

  // 设置初始状态
  state_.current_state = DecisionState::IDLE;
  state_.last_state_change_time = get_current_time();

  log_info("导航系统初始化完成");
}

void DecisionTree::set_parameters()
{
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
  set_params(this, "no_attack_duration", pd.no_attack_duration, 15.0);   // 15秒
  set_params(this, "approach_radius", pd.approach_radius, 4.0);          // 4米

  // 位置参数
  set_params(this, "base_position_x", pd.base_x, 0.0);
  set_params(this, "base_position_y", pd.base_y, 0.0);
  set_params(this, "center_position_x", pd.center_x, 0.0);
  set_params(this, "center_position_y", pd.center_y, 0.0);

  log_info("参数加载完成:");
  log_info("  机器人基础坐标系: " + params_.robot_base_frame);
  log_info("  地图话题: " + params_.map_topic);
  log_info("  位置容差: " + std::to_string(params_.position_tolerance) + "m");
  log_info("  基地位置: (" + std::to_string(params_.decision_params.base_x) + ", " +
           std::to_string(params_.decision_params.base_y) + ")");
  log_info("  增益点位置: (" + std::to_string(params_.decision_params.center_x) + ", " +
           std::to_string(params_.decision_params.center_y) + ")");
}

void DecisionTree::calculate_enemy_base_position()
{
  // 假设敌方基地在增益点的对称位置
  params_.decision_params.enemy_base_x = 2 * params_.decision_params.center_x - params_.decision_params.base_x;
  params_.decision_params.enemy_base_y = 2 * params_.decision_params.center_y - params_.decision_params.base_y;

  log_info("计算敌方基地位置: (" + std::to_string(params_.decision_params.enemy_base_x) + ", " +
           std::to_string(params_.decision_params.enemy_base_y) + ")");
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

  // RFID状态订阅
  rfid_status_sub_ = this->create_subscription<connection_layer::msg::RfidStatus>(
      "rfid_status", rclcpp::QoS(10).reliable(),
      [this](const connection_layer::msg::RfidStatus::SharedPtr msg) { rfid_status_callback(msg); });

  // 机器人状态订阅
  robot_status_sub_ = this->create_subscription<connection_layer::msg::RobotStatus>(
      "robot_status", rclcpp::QoS(10).reliable(),
      [this](const connection_layer::msg::RobotStatus::SharedPtr msg) { robot_status_callback(msg); });

  log_info("所有订阅器初始化完成");
}

void DecisionTree::initialize_timers()
{
  // 状态更新定时器
  status_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { update_status(); });

  // 连接检查定时器
  connection_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
    check_connection();
    connection_timer_->cancel();
  });

  // 决策树定时器
  decision_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { decision_tree_callback(); });

  // 攻击对策定时器
  attack_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { attack_check_callback(); });

  log_info("定时器初始化完成");
}

void DecisionTree::initialize_navigation()
{
  nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  log_info("导航动作客户端初始化完成");
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

    log_info("接收到地图数据:");
    log_info("  尺寸: " + std::to_string(msg->info.width) + "x" + std::to_string(msg->info.height));
    log_info("  分辨率: " + std::to_string(msg->info.resolution));
    log_info("  原点: (" + std::to_string(msg->info.origin.position.x) + ", " +
             std::to_string(msg->info.origin.position.y) + ")");
  }
  catch (const std::exception& e)
  {
    log_error("处理地图数据失败: " + std::string(e.what()));
    state_.map_available = false;
  }
}

void DecisionTree::game_status_callback(const connection_layer::msg::GameStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);

  // 更新游戏状态
  game_state_.game_progress = msg->game_progress;
  game_state_.stage_remain_time = msg->stage_remain_time;

  // 根据比赛进程调整决策状态
  switch (game_state_.game_progress)
  {
    case 0:  // 未开始
      if (state_.current_state != DecisionState::WAITING_FOR_START)
      {
        change_state(DecisionState::WAITING_FOR_START);
        log_info("比赛未开始，进入等待状态");
      }
      break;

    case 1:  // 进行中
      if (state_.current_state == DecisionState::WAITING_FOR_START || state_.current_state == DecisionState::IDLE)
      {
        change_state(DecisionState::CAPTURING_CENTER);
        log_info("比赛开始，进入抢占增益点状态");
      }
      break;

    case 2:  // 结束
      change_state(DecisionState::PAUSED);
      cancel_navigation();
      log_info("比赛结束，停止所有行动");
      break;
  }

  log_debug("游戏状态更新 - 进程: " + std::to_string(game_state_.game_progress) +
            ", 剩余时间: " + std::to_string(game_state_.stage_remain_time) + "秒");
}

void DecisionTree::rfid_status_callback(const connection_layer::msg::RfidStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);

  game_state_.center_gain_point = msg->center_gain_point;

  log_debug("增益点状态更新: " + std::to_string(game_state_.center_gain_point));
}

void DecisionTree::robot_status_callback(const connection_layer::msg::RobotStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);

  // 记录血量变化
  static uint16_t previous_hp = game_state_.current_hp;
  uint16_t current_hp = msg->current_hp;

  if (current_hp < previous_hp)
  {
    // 血量减少，说明受到攻击
    game_state_.last_attack_time = get_current_time();
    game_state_.under_attack = true;
    log_info("检测到受到攻击! 当前血量: " + std::to_string(current_hp) + ", 上次血量: " + std::to_string(previous_hp));
  }

  // 更新血量
  previous_hp = current_hp;
  game_state_.current_hp = current_hp;
  game_state_.maximum_hp = msg->maximum_hp;

  log_debug("机器人状态更新 - 血量: " + std::to_string(game_state_.current_hp) + "/" +
            std::to_string(game_state_.maximum_hp) + " (" + std::to_string(get_hp_percentage() * 100) + "%)");
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
  status_msg += ", 地图可用=" + std::to_string(state_.map_available);
  status_msg += ", TF可用=" + std::to_string(state_.tf_available);

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

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s", status_msg.c_str());
}

void DecisionTree::check_connection()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  log_info("执行连接检查...");

  // 检查TF连接
  bool tf_ok = check_tf_connection();
  if (!tf_ok)
  {
    log_warning("TF连接检查失败");
  }
  else
  {
    state_.tf_available = true;
    log_info("TF连接正常");
  }

  // 检查导航服务器
  bool nav_ok = check_navigation_server();
  if (!nav_ok)
  {
    log_warning("导航服务器连接检查失败");
  }
  else
  {
    log_info("导航服务器连接正常");
  }

  if (tf_ok && nav_ok)
  {
    state_.is_active = true;
    log_info("所有连接检查通过，系统准备就绪");
  }
  else
  {
    log_warning("连接检查未完全通过，部分功能可能受限");
  }
}

void DecisionTree::decision_tree_callback()
{
  if (!state_.is_active)
  {
    return;
  }

  // 检查比赛进程
  {
    std::lock_guard<std::mutex> lock(game_state_mutex_);
    if (game_state_.game_progress != 1)
    { 
      // 如果当前不是等待状态，且比赛未开始或已结束，则暂停
      if (state_.current_state != DecisionState::WAITING_FOR_START && state_.current_state != DecisionState::PAUSED &&
          state_.current_state != DecisionState::IDLE)
      {
        change_state(DecisionState::PAUSED);
        cancel_navigation();
      }
      return;
    }
  }

  // 更新决策状态
  update_decision_state();

  // 执行当前状态
  execute_decision_state();

  state_.last_decision_time = get_current_time();
}

void DecisionTree::attack_check_callback()
{
  if (game_state_.under_attack == false)
    return;

  /// @brief 逻辑待处理

  game_state_.under_attack = false;
}

// =============== 导航方法 ===============

bool DecisionTree::navigate_to_point(double x, double y, double yaw)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return navigate_to_point_impl(x, y, yaw);
}

bool DecisionTree::navigate_to_point_impl(double x, double y, double yaw)
{
  // 检查目标点是否在地图边界内
  if (state_.map_available && !is_in_map_boundary(x, y))
  {
    log_error("目标点不在有效地图边界内");
    return false;
  }

  // 检查当前位置与目标位置的距离
  if (get_robot_position())
  {
    double distance = calculate_distance(state_.robot_x, state_.robot_y, x, y);
    if (distance < params_.position_tolerance)
    {
      log_info("已在目标点附近 (距离: " + std::to_string(distance) + "m), 跳过导航");
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

  send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr& goal_handle) {
        goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr>(
            std::async(std::launch::deferred, [goal_handle]() { return goal_handle; }).share()));
      };

  send_goal_options.feedback_callback = [this](GoalHandleNavigateToPose::SharedPtr,
                                               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    nav_feedback_callback(nullptr, feedback);
  };

  send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult& result) {
    nav_result_callback(result);
  };

  auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);

  state_.is_navigating = true;
  state_.target_x = pose.pose.position.x;
  state_.target_y = pose.pose.position.y;

  log_info("开始导航到目标点: (" + std::to_string(pose.pose.position.x) + ", " + std::to_string(pose.pose.position.y) +
           ")");

  return true;
}

void DecisionTree::cancel_navigation()
{
  if (current_goal_handle_)
  {
    log_info("取消当前导航");
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
    // 检查TF变换是否可用
    if (!tf_buffer_->canTransform("map", params_.robot_base_frame, tf2::TimePointZero))
    {
      state_.tf_available = false;
      return false;
    }

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

    state_.tf_available = true;
    return true;
  }
  catch (const tf2::TransformException& e)
  {
    log_warning("获取机器人位置失败: " + std::string(e.what()));
    state_.tf_available = false;
    return false;
  }
  catch (const std::exception& e)
  {
    log_error("获取位置异常: " + std::string(e.what()));
    state_.tf_available = false;
    return false;
  }
}

// =============== 导航回调 ===============

void DecisionTree::goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle)
  {
    log_error("导航目标被拒绝");

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.is_navigating = false;
    return;
  }

  log_info("导航目标已接受");
  current_goal_handle_ = goal_handle;
}

void DecisionTree::nav_feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                                         const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "导航进度 - 剩余距离: %.2f米, 导航时间: %d秒",
                       feedback->distance_remaining,
                       feedback->navigation_time.sec);
}

void DecisionTree::nav_result_callback(const GoalHandleNavigateToPose::WrappedResult& result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      log_info("导航成功完成");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      log_warning("导航被中止");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      log_info("导航被取消");
      break;
    default:
      log_error("未知导航结果");
      break;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.is_navigating = false;
  current_goal_handle_.reset();

  /// @brief 逻辑待完善 导航完成后，根据当前状态决定下一步
  if (state_.current_state == DecisionState::NAVIGATING_TO_POINT)
  {
    change_state(DecisionState::CAPTURING_CENTER);
  }
}

// =============== 连接检查 ===============

bool DecisionTree::check_tf_connection()
{
  try
  {
    if (!tf_buffer_->canTransform("map", params_.robot_base_frame, tf2::TimePointZero))
    {
      log_warning("缺少从'map'到'" + params_.robot_base_frame + "'的变换");
      return false;
    }

    tf_buffer_->lookupTransform("map", params_.robot_base_frame, tf2::TimePointZero);
    return true;
  }
  catch (const tf2::TransformException& e)
  {
    log_warning("TF连接检查失败: " + std::string(e.what()));
    return false;
  }
  catch (const std::exception& e)
  {
    log_error("TF连接检查异常: " + std::string(e.what()));
    return false;
  }
}

bool DecisionTree::check_navigation_server()
{
  try
  {
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(static_cast<int>(params_.connection_timeout))))
    {
      log_warning("导航服务器未响应（超时: " + std::to_string(params_.connection_timeout) + "s）");
      return false;
    }
    return true;
  }
  catch (const std::exception& e)
  {
    log_error("导航服务器检查异常: " + std::string(e.what()));
    return false;
  }
}

// =============== 决策树方法 ===============

void DecisionTree::update_decision_state()
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);

  // 规则1: 当前血量低时返回基地补充
  if (is_low_hp() && state_.current_state != DecisionState::RETURNING_TO_BASE)
  {
    change_state(DecisionState::RETURNING_TO_BASE);
    return;
  }

  // 规则2: 正在返回基地，血量补充到90%以上继续行动
  if (state_.current_state == DecisionState::RETURNING_TO_BASE && is_safe_hp())
  {
    change_state(DecisionState::CAPTURING_CENTER);
    return;
  }

  // 规则3: 如果不在其他特殊状态，检查是否需要抢占增益点
  if (state_.current_state != DecisionState::RETURNING_TO_BASE &&
      state_.current_state != DecisionState::APPROACHING_ENEMY && !is_center_captured())
  {
    change_state(DecisionState::CAPTURING_CENTER);
    return;
  }

  // 规则4: 当血量充足（50%以上）且增益点为占领状态，15秒未受到攻击，尝试接近敌方基地
  if (is_center_captured() && is_normal_hp() && has_not_been_attacked_for(params_.decision_params.no_attack_duration) &&
      state_.current_state != DecisionState::APPROACHING_ENEMY)
  {
    change_state(DecisionState::APPROACHING_ENEMY);
    return;
  }

  // 规则5: 如果正在接近敌方基地但条件不再满足，返回抢占中心点
  if (state_.current_state == DecisionState::APPROACHING_ENEMY &&
      (!is_normal_hp() || !is_center_captured() ||
       !has_not_been_attacked_for(params_.decision_params.no_attack_duration)))
  {
    change_state(DecisionState::CAPTURING_CENTER);
    return;
  }
}

void DecisionTree::execute_decision_state()
{
  switch (state_.current_state)
  {
    case DecisionState::IDLE:
      handle_idle_state();
      break;
    case DecisionState::WAITING_FOR_START:
      handle_waiting_state();
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
    case DecisionState::NAVIGATING_TO_POINT:
      handle_navigating_state();
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

    log_info("决策状态改变: " + state_to_string(state_.previous_state) + " -> " +
             state_to_string(state_.current_state));

    // 状态改变时取消当前导航
    if (state_.previous_state == DecisionState::CAPTURING_CENTER ||
        state_.previous_state == DecisionState::RETURNING_TO_BASE ||
        state_.previous_state == DecisionState::APPROACHING_ENEMY ||
        state_.previous_state == DecisionState::NAVIGATING_TO_POINT)
    {
      cancel_navigation();
    }
  }
}

void DecisionTree::handle_idle_state()
{
  /// @brief 待处理
}

void DecisionTree::handle_waiting_state()
{
  std::lock_guard<std::mutex> lock(game_state_mutex_);
  if (game_state_.game_progress == 1)
  {
    change_state(DecisionState::CAPTURING_CENTER);
  }
}

void DecisionTree::handle_capturing_center_state()
{
  if (!state_.is_navigating)
  {
    log_info("开始抢占增益点");
    navigate_to_point_impl(params_.decision_params.center_x, params_.decision_params.center_y);
  }

  // 检查是否到达增益点
  if (state_.is_navigating)
  {
    double distance = calculate_distance(state_.robot_x, state_.robot_y, params_.decision_params.center_x,
                                         params_.decision_params.center_y);

    if (distance < params_.position_tolerance)
    {
      log_info("已到达增益点附近");
      /// @brief 待处理
    }
  }
}

void DecisionTree::handle_returning_base_state()
{
  if (!state_.is_navigating)
  {
    log_info("返回基地补充血量");
    navigate_to_point_impl(params_.decision_params.base_x, params_.decision_params.base_y);
  }

  if (state_.is_navigating)
  {
    double distance = calculate_distance(state_.robot_x, state_.robot_y, params_.decision_params.base_x,
                                         params_.decision_params.base_y);

    if (distance < params_.position_tolerance)
    {
      log_info("已到达基地");
      /// @brief 待处理
    }
  }
}

void DecisionTree::handle_approaching_enemy_state()
{
  if (!state_.is_navigating)
  {
    auto approach_point = calculate_approach_point();
    log_info("开始接近敌方基地，目标点: (" + std::to_string(approach_point.first) + ", " +
             std::to_string(approach_point.second) + ")");

    navigate_to_point_impl(approach_point.first, approach_point.second);
    change_state(DecisionState::NAVIGATING_TO_POINT);
  }
}

void DecisionTree::handle_navigating_state()
{
  /// @brief 待处理
}

// =============== 决策辅助方法 ===============

bool DecisionTree::is_center_captured() const
{
  // 假设增益点状态为非0表示已占领
  return game_state_.center_gain_point != 0;
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
  return static_cast<double>(game_state_.current_hp) / game_state_.maximum_hp;
}

// =============== 工具函数 ===============

std::pair<double, double> DecisionTree::calculate_approach_point() const
{
  double dx = params_.decision_params.enemy_base_x - params_.decision_params.center_x;
  double dy = params_.decision_params.enemy_base_y - params_.decision_params.center_y;

  // 计算到敌方基地的距离
  double distance_to_enemy = std::sqrt(dx * dx + dy * dy);

  // 如果距离小于approach_radius，直接使用敌方基地位置
  if (distance_to_enemy <= params_.decision_params.approach_radius)
  {
    return { params_.decision_params.enemy_base_x, params_.decision_params.enemy_base_y };
  }

  // 否则在连线上选择一个点
  double ratio = params_.decision_params.approach_radius / distance_to_enemy;
  double approach_x = params_.decision_params.center_x + dx * ratio;
  double approach_y = params_.decision_params.center_y + dy * ratio;

  return { approach_x, approach_y };
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

  if (!state_.map_available || !map_metadata_)
  {
    log_warning("地图数据不可用，无法验证边界");
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

void DecisionTree::log_info(const std::string& message) const
{
  RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
}

void DecisionTree::log_warning(const std::string& message) const
{
  RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
}

void DecisionTree::log_error(const std::string& message) const
{
  RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
}

void DecisionTree::log_debug(const std::string& message) const
{
  RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
}

std::string DecisionTree::state_to_string(DecisionState state) const
{
  switch (state)
  {
    case DecisionState::IDLE:
      return "IDLE";
    case DecisionState::WAITING_FOR_START:
      return "WAITING_FOR_START";
    case DecisionState::CAPTURING_CENTER:
      return "CAPTURING_CENTER";
    case DecisionState::RETURNING_TO_BASE:
      return "RETURNING_TO_BASE";
    case DecisionState::APPROACHING_ENEMY:
      return "APPROACHING_ENEMY";
    case DecisionState::NAVIGATING_TO_POINT:
      return "NAVIGATING_TO_POINT";
    case DecisionState::PAUSED:
      return "PAUSED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace decision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decision::DecisionTree)
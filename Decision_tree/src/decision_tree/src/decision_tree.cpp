#include "decision_tree/decision_tree.hpp"

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
  set_params(this, "map_topic", p.map_topic, std::string("map"));
  set_params(this, "goal_topic", p.goal_topic, std::string("goal_topic"));
  set_params(this, "position_tolerance", p.position_tolerance, 0.3);
  set_params(this, "maximum_hp", p.maximum_hp, 300);
  set_params(this, "chase_hp", p.chase_hp, 50);
  set_params(this, "chase_proportion", p.chase_proportion, 0.6);
  set_params(this, "approaching_enemy_x", p.approaching_enemy_x, 0.9);
  set_params(this, "approaching_enemy_y", p.approaching_enemy_y, 0.9);

  // 决策参数
  set_params(this, "low_hp_threshold", pd.low_hp_threshold, 0.3);        // 30%
  set_params(this, "safe_hp_threshold", pd.safe_hp_threshold, 0.9);      // 90%
  set_params(this, "normal_hp_threshold", pd.normal_hp_threshold, 0.5);  // 50%
  set_params(this, "under_attack_delay", pd.under_attack_delay, 5.0);    // 5秒

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
  state_.last_state_change_time = get_current_time();

  logger("导航系统初始化完成", "info");
}

void DecisionTree::initialize_subscriptions()
{
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
  status_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() { update_status(); });

  // 决策树定时器
  decision_timer_ = this->create_wall_timer(std::chrono::milliseconds(300), [this]() { decision_tree_callback(); });

  // 攻击对策定时器
  attack_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(300), [this]() { attack_check_callback(); });
}

void DecisionTree::initialize_navigation()
{
  nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, params_.goal_topic);
}

// =============== 回调函数 ===============

void DecisionTree::game_status_callback(const connection_layer::msg::GameStatus::SharedPtr msg)
{
  auto game_lock = write_lock(game_state_mutex_);

  // 更新游戏状态
  game_state_.game_progress = static_cast<int>(msg->progress);
  game_state_.stage_remain_time = msg->remaining;

  game_state_.center_gain_point = static_cast<int>(msg->gain);

  // 根据比赛进程调整决策状态
  if (game_state_.game_progress == 4 && !state_.is_active)
  {
    logger("比赛开始", "info");
    state_.is_active = true;
  }
  else if (game_state_.game_progress == 5)
  {
    logger("比赛结束", "warn");
    state_.is_active = false;
    state_.current_state = DecisionState::PAUSED;
    state_.previous_state = DecisionState::PAUSED;
  }

  logger("游戏状态更新 - 进程: " + std::to_string(game_state_.game_progress) +
             ", 剩余时间: " + std::to_string(game_state_.stage_remain_time) + "秒",
         "debug");
}

void DecisionTree::robot_status_callback(const connection_layer::msg::RobotStatus::SharedPtr msg)
{
  auto game_lock = write_lock(game_state_mutex_);

  // 记录血量变化
  int previous_hp = game_state_.current_hp;
  int current_hp = static_cast<int>(msg->hp);

  if (current_hp < previous_hp)
  {
    // 血量减少，说明受到攻击
    game_state_.last_attack_time = get_current_time();
    logger("检测到受到攻击! 当前血量: " + std::to_string(current_hp) + ", 上次血量: " + std::to_string(previous_hp),
           "info");
    game_state_.under_attack = true;
  }

  if (get_current_time() - game_state_.last_attack_time > params_.decision_params.under_attack_delay)
  {
    game_state_.under_attack = false;
  }

  // 更新血量
  game_state_.current_hp = current_hp;

  auto attack_lock = write_lock(attack_state_mutex_);
  attack_state_.infantry_hp = msg->infantry_hp;
  attack_state_.hero_hp = msg->hero_hp;
  attack_state_.sentinel_hp = msg->sentinel_hp;

  logger("机器人状态更新 - 血量: " + std::to_string(game_state_.current_hp) + "/" + std::to_string(params_.maximum_hp) +
             " (" + std::to_string(get_hp_percentage() * 100) + "%)",
         "debug");
}

// =============== 定时器回调 ===============

void DecisionTree::update_status()
{
  // 更新机器人位置
  std::vector<double> position = get_robot_position();

  if (position[0] > params_.decision_params.activate_search)
  {
    state_.search = true;
  }
  else if (!game_state_.under_attack)
  {
    state_.rotate = false;
    state_.search = false;
  }
  if (!state_.is_active)
  {
    state_.search = false;
    state_.rotate = false;
  }
  pub_control_signal(state_.rotate, state_.search);

  auto lock = write_lock(position_mutex_);
  state_.position = std::move(position);
}

void DecisionTree::decision_tree_callback()
{
  if (!state_.is_active)
  {
    return;
  }

  auto new_state = normal_decision();
  if (!state_.is_chase)
    change_state(new_state);
  else
    return;
}

void DecisionTree::attack_check_callback()
{
  {
    auto game_lock = read_lock(game_state_mutex_);
    auto attack_lock = read_lock(attack_state_mutex_);
    if (!state_.is_active)
    {
      return;
    }
    else if (!game_state_.under_attack || !attack_state_.lock_enemy)
      return;
  }
  auto new_state = attack_decision();
  if (state_.is_chase)
    change_state(new_state);
}

// =============== 导航方法 ===============

bool DecisionTree::navigate_to_point(double x, double y, double yaw)
{
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
        }
        else
        {
          logger("导航目标已接受", "info");
          current_goal_handle_ = goal_handle;
        }
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

    current_goal_handle_.reset();
  };

  auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);

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

  current_goal_handle_.reset();
}

std::vector<double> DecisionTree::get_robot_position()
{
  try
  {
    // 获取变换
    auto tf = tf_buffer_->lookupTransform(params_.map_topic, params_.robot_base_frame, tf2::TimePointZero);
    std::vector<double> position(3);
    position[0] = tf.transform.translation.x;
    position[1] = tf.transform.translation.y;

    // 计算偏航角
    double qx = tf.transform.rotation.x;
    double qy = tf.transform.rotation.y;
    double qz = tf.transform.rotation.z;
    double qw = tf.transform.rotation.w;

    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    position[2] = yaw;
    return position;

    logger("位置获取: x=" + std::to_string(position[0]) + ", y=" + std::to_string(position[1]), "warn");
  }
  catch (const tf2::TransformException& e)
  {
    logger("获取机器人位置失败: " + std::string(e.what()), "warn");
    return state_.position;
  }
  catch (const std::exception& e)
  {
    logger("获取位置异常: " + std::string(e.what()), "error");
    return state_.position;
  }
}

// =============== 决策树方法 ===============
void DecisionTree::change_state(DecisionState new_state)
{
  if (state_.current_state != new_state)
  {
    state_.previous_state = state_.current_state;
    state_.current_state = new_state;
    state_.last_state_change_time = get_current_time();

    logger("决策状态改变: " + state_to_string(state_.previous_state) + " -> " + state_to_string(state_.current_state),
           "info");

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
      case DecisionState::CHASE_ENEMY:
        handle_chase_enemy_state();
        break;
      case DecisionState::PAUSED:
        // 暂停状态，不执行任何操作
        break;
    }
  }
}

DecisionState DecisionTree::normal_decision()
{
  auto lock = read_lock(game_state_mutex_);
  // 规则: 当前血量低时返回基地补充
  if (is_low_hp())
  {
    return DecisionState::RETURNING_TO_BASE;
  }

  // 规则: 返回基地状态，血量补充到安全继续行动
  else if (state_.current_state == DecisionState::RETURNING_TO_BASE)
  {
    if (is_safe_hp())
    {
      return DecisionState::CAPTURING_CENTER;
    }
    return DecisionState::RETURNING_TO_BASE;
  }

  // 规则：己方未占领则执行占领
  else if (!center_captured())
  {
    return DecisionState::CAPTURING_CENTER;
  }

  // 规则: 如果不在增益点并且己方占领
  else if (!in_key_position(1) && center_captured())
  {
    return DecisionState::APPROACHING_ENEMY;
  }

  // 规则: 当血量充足且增益点为己方占领状态，未受到攻击，尝试接近敌方
  else if (is_normal_hp() && center_captured() &&
           has_not_been_attacked_for(params_.decision_params.under_attack_delay) &&
           state_.current_state != DecisionState::APPROACHING_ENEMY)
  {
    return DecisionState::APPROACHING_ENEMY;
  }

  // 规则: 如果正在接近敌方基地但条件不再满足，返回抢占中心点
  else if (state_.current_state == DecisionState::APPROACHING_ENEMY && !center_captured())
  {
    return DecisionState::CAPTURING_CENTER;
  }

  return DecisionState::GUARD;
}

DecisionState DecisionTree::attack_decision()
{
  auto lock = read_lock(game_state_mutex_);
  auto attack_lock = read_lock(attack_state_mutex_);
  // 规则：自身血量正常，且敌方血量低于阈值追击
  if (is_normal_hp() && is_chase_hp(attack_state_.enemy_type))
  {
    state_.is_chase = true;
    state_.rotate = false;
    return DecisionState::CHASE_ENEMY;
  }
  // 规则：受击立刻开启小陀螺
  else if (game_state_.under_attack)
  {
    state_.is_chase = true;
    state_.rotate = true;
  }

  state_.is_chase = false;
  return state_.current_state;
}

void DecisionTree::handle_guard_state()
{
  state_.rotate = true;
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
  double x = params_.decision_params.center_x * params_.approaching_enemy_x;
  double y = params_.decision_params.base_y * params_.approaching_enemy_y;
  logger("压制敌方，目标点: (" + std::to_string(x) + ", " + std::to_string(y) + ")", "info");

  navigate_to_point(x, y);
}

void DecisionTree::handle_chase_enemy_state()
{
  double enemy_x, enemy_y;
  {
    std::vector<double> position;
    {
      auto lock = read_lock(position_mutex_);
      position = state_.position;
    }
    enemy_x = position[0] + params_.chase_proportion * attack_state_.enemy_distance * std::cos(position[2]);
    enemy_y = position[1] + params_.chase_proportion * attack_state_.enemy_distance * std::sin(position[2]);
  }
  logger("追击模式启动", "info");
  navigate_to_point(enemy_x, enemy_y);
}

// =============== 决策辅助方法 ===============

bool DecisionTree::in_key_position(const int key)
{
  std::vector<double> position;
  {
    auto lock = read_lock(position_mutex_);
    position = state_.position;
  }
  if (key == 0)
  {
    double distance =
        calculate_distance(position[0], position[1], params_.decision_params.base_x, params_.decision_params.base_y);

    if (distance < params_.position_tolerance)
    {
      logger("已在基地", "info");
      return true;
    }
  }
  else if (key == 1)
  {
    double distance = calculate_distance(position[0], position[1], params_.decision_params.center_x,
                                         params_.decision_params.center_y);

    if (distance < params_.position_tolerance)
    {
      logger("已在增益点", "info");
      state_.rotate = true;
      return true;
    }
  }
  return false;
}

bool DecisionTree::has_not_been_attacked_for(double duration)
{
  double current_time = get_current_time();
  return (current_time - game_state_.last_attack_time) >= duration;
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

bool DecisionTree::is_chase_hp(int type) const
{
  if (type == 0)
    return attack_state_.infantry_hp <= params_.chase_hp;
  else if (type == 1)
    return attack_state_.hero_hp <= params_.chase_hp;
  else
    return attack_state_.sentinel_hp <= params_.chase_hp;
}

double DecisionTree::get_hp_percentage() const
{
  return static_cast<double>(game_state_.current_hp) / params_.maximum_hp;
}

// =============== 工具函数 ===============

void DecisionTree::pub_control_signal(const bool rotate, const bool search) const
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
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance;
}

double DecisionTree::get_current_time()
{
  return this->get_clock()->now().seconds();
}

std::shared_lock<std::shared_mutex> DecisionTree::read_lock(std::shared_mutex& lock) const
{
  return std::shared_lock<std::shared_mutex>(lock);
}

std::unique_lock<std::shared_mutex> DecisionTree::write_lock(std::shared_mutex& lock) const
{
  return std::unique_lock<std::shared_mutex>(lock);
}

void DecisionTree::logger(const std::string& message, const std::string type) const
{
  if (type == "info")
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  else if (type == "warn")
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
  else if (type == "error")
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
  else if (type == "debug")
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
    case DecisionState::CHASE_ENEMY:
      return "CHASE_ENEMY";
    case DecisionState::PAUSED:
      return "PAUSED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace decision_tree

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(decision_tree::DecisionTree)
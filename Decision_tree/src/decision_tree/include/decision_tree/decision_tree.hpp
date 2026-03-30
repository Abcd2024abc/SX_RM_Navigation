#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 添加游戏状态消息
#include "connection_layer/msg/game_status.hpp"
#include "connection_layer/msg/robot_status.hpp"
#include "connection_layer/msg/control_signal.hpp"

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <mutex>
#include <atomic>
#include <functional>

namespace decision_tree
{
// 决策状态枚举
enum class DecisionState
{
  GUARD,              // 守卫状态
  CAPTURING_CENTER,   // 抢占增益点
  RETURNING_TO_BASE,  // 返回基地补充
  APPROACHING_ENEMY,  // 接近敌方
  CHASE_ENEMY,        // 追击模式
  PAUSED              // 暂停状态
};

// 参数结构体
struct Parameters
{
  std::string robot_base_frame = "base_footprint";
  std::string map_topic = "map";
  std::string goal_topic;
  double position_tolerance = 0.3;
  int maximum_hp = 300;
  int chase_hp = 50;
  double chase_proportion = 0.6;
  double approaching_enemy_x = 0.9;
  double approaching_enemy_y = 0.9;

  // 决策参数
  struct DecisionParams
  {
    double low_hp_threshold = 0.3;     // 低血量阈值（30%）
    double safe_hp_threshold = 0.9;    // 安全血量阈值（90%）
    double normal_hp_threshold = 0.5;  // 正常血量阈值（50%）
    double under_attack_delay = 5.0;   // 未受攻击持续时间（秒）
    double base_x = 0.0;               // 基地X坐标
    double base_y = 0.0;               // 基地Y坐标
    double center_x = 0.0;             // 增益点X坐标
    double center_y = 0.0;             // 增益点Y坐标

    float activate_search = 5.0;

    DecisionParams() = default;
  } decision_params;
};

// 游戏状态
struct GameState
{
  int game_progress;        // 比赛进程(0=未开始,1=准备,2=自检，3=倒计时，4=比赛中，5=结束)
  float stage_remain_time;  // 当前阶段剩余时间
  int center_gain_point;    // 增益点状态(0=未占领，1=己方占领，2=敌方占领，3=争夺)
  int current_hp;           // 当前血量

  std::atomic<bool> under_attack;  // 遭到攻击
  double last_attack_time;         // 上次受到攻击时间

  GameState() : game_progress(0), stage_remain_time(-1.0), center_gain_point(-1), current_hp(-1), under_attack(false)
  {
  }
};

// 机器人状态
struct AttackState
{
  std::atomic<bool> lock_enemy;  // 锁定敌方

  float enemy_distance;  // 敌方距离
  int enemy_type;        // 敌方类型

  int infantry_hp;  // 敌方步兵
  int hero_hp;      // 敌方英雄
  int sentinel_hp;  // 敌方哨兵
  AttackState() : lock_enemy(false), infantry_hp(-1), hero_hp(-1), sentinel_hp(-1)
  {
  }
};

// 条件状态
struct ConditionState
{
  std::atomic<bool> is_chase = false;
  std::atomic<bool> is_active = false;
  std::atomic<bool> rotate = false;
  std::atomic<bool> search = false;
  std::vector<double> position = { 0.0, 0.0, 0.0 };

  // 决策状态
  DecisionState current_state = DecisionState::PAUSED;
  DecisionState previous_state = DecisionState::PAUSED;
  double last_state_change_time = 0.0;

  ConditionState() = default;
};

class DecisionTree : public rclcpp::Node
{
public:
  explicit DecisionTree(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DecisionTree();

  void initialize();

  bool navigate_to_point(double x, double y, double yaw = 0.0);
  void cancel_navigation();

private:
  // 参数
  Parameters params_;
  // 条件状态
  ConditionState state_;
  // 游戏状态
  GameState game_state_;
  // 攻击状态
  AttackState attack_state_;

  // TF相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 导航动作客户端
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr current_goal_handle_;

  // 发布者
  rclcpp::Publisher<connection_layer::msg::ControlSignal>::SharedPtr control_pub_;

  // 订阅器

  // 游戏状态订阅器
  rclcpp::Subscription<connection_layer::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<connection_layer::msg::RobotStatus>::SharedPtr robot_status_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;
  rclcpp::TimerBase::SharedPtr decision_timer_;
  rclcpp::TimerBase::SharedPtr attack_check_timer_;

  // 互斥锁
  mutable std::shared_mutex position_mutex_;
  mutable std::shared_mutex attack_state_mutex_;
  mutable std::shared_mutex game_state_mutex_;

  // 初始化方法
  template <class T>
  void set_params(rclcpp::Node* node, const std::string& params_name, T& target, const T& default_val)
  {
    node->declare_parameter<T>(params_name, default_val);
    node->get_parameter(params_name, target);
  }
  void initialize_subscriptions();
  void initialize_timers();
  void initialize_navigation();

  // 回调函数
  void game_status_callback(const connection_layer::msg::GameStatus::SharedPtr msg);
  void robot_status_callback(const connection_layer::msg::RobotStatus::SharedPtr msg);

  // 定时器回调
  void update_status();
  void decision_tree_callback();
  void attack_check_callback();

  // 导航方法
  bool navigate_to_pose(const geometry_msgs::msg::PoseStamped& pose);
  std::vector<double> get_robot_position();

  // 决策树方法
  void change_state(DecisionState new_state);
  DecisionState normal_decision();
  DecisionState attack_decision();
  void handle_guard_state();
  void handle_capturing_center_state();
  void handle_returning_base_state();
  void handle_approaching_enemy_state();
  void handle_chase_enemy_state();

  // 决策辅助方法
  bool in_key_position(const int key);
  bool has_not_been_attacked_for(double duration);
  bool center_captured() const;
  bool is_low_hp() const;
  bool is_safe_hp() const;
  bool is_normal_hp() const;
  bool is_chase_hp(int type) const;
  double get_hp_percentage() const;

  // 工具函数
  void pub_control_signal(const bool rotate, const bool search) const;
  double calculate_distance(double x1, double y1, double x2, double y2) const;
  double get_current_time();
  std::shared_lock<std::shared_mutex> read_lock(std::shared_mutex& lock) const;
  std::unique_lock<std::shared_mutex> write_lock(std::shared_mutex& lock) const;

  // 日志辅助
  void logger(const std::string& message, const std::string type) const;

  // 状态字符串转换
  std::string state_to_string(DecisionState state) const;
};

}  // namespace decision_tree

#ifndef DECISION_TREE_HPP
#define DECISION_TREE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp_action/rclcpp_action.hpp>

// 添加游戏状态消息
#include "connection_layer/msg/game_status.hpp"
#include "connection_layer/msg/robot_status.hpp"

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>
#include <mutex>
#include <atomic>
#include <functional>

namespace decision_tree
{

// 决策状态枚举
enum class DecisionState
{
  IDLE,                 // 空闲状态
  WAITING_FOR_START,    // 等待比赛开始
  CAPTURING_CENTER,     // 抢占增益点
  RETURNING_TO_BASE,    // 返回基地补充
  APPROACHING_ENEMY,    // 接近敌方基地
  NAVIGATING_TO_POINT,  // 导航到指定点
  PAUSED                // 暂停状态
};

// 游戏状态
struct GameState
{
  uint8_t game_progress;       // 比赛进程
  uint16_t stage_remain_time;  // 当前阶段剩余时间
  uint32_t center_gain_point;  // 增益点状态
  uint16_t current_hp;         // 当前血量
  uint16_t maximum_hp;         // 最大血量
  double last_attack_time;     // 上次受到攻击时间
  bool under_attack;

  GameState()
    : game_progress(0), stage_remain_time(0), center_gain_point(0), current_hp(0), maximum_hp(0), last_attack_time(0.0)
  {
    under_attack = false;
  }
};

class DecisionTree : public rclcpp::Node
{
public:
  explicit DecisionTree(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DecisionTree();

  void initialize();

  bool navigate_to_point(double x, double y, double yaw = 0.0);
  void cancel_navigation();
  bool is_navigating() const
  {
    return state_.is_navigating;
  }

private:
  // 参数结构体
  struct Parameters
  {
    std::string robot_base_frame = "base_footprint";
    std::string map_topic = "/map";
    double position_tolerance = 0.3;
    double navigation_timeout = 60.0;
    double connection_timeout = 5.0;
    int max_connection_retries = 3;

    // 决策参数
    struct DecisionParams
    {
      double low_hp_threshold = 0.3;     // 低血量阈值（30%）
      double safe_hp_threshold = 0.9;    // 安全血量阈值（90%）
      double normal_hp_threshold = 0.5;  // 正常血量阈值（50%）
      double no_attack_duration = 15.0;  // 未受攻击持续时间（秒）
      double approach_radius = 4.0;      // 接近敌方基地半径（米）
      double base_x = 0.0;               // 基地X坐标
      double base_y = 0.0;               // 基地Y坐标
      double center_x = 0.0;             // 增益点X坐标
      double center_y = 0.0;             // 增益点Y坐标
      double enemy_base_x = 0.0;         // 敌方基地X坐标
      double enemy_base_y = 0.0;         // 敌方基地Y坐标

      DecisionParams() = default;
    } decision_params;
  } params_;

  // 导航状态
  struct NavigationState
  {
    bool is_active = false;
    bool is_navigating = false;
    double target_x = 0.0;
    double target_y = 0.0;
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    bool map_available = false;
    bool tf_available = false;

    // 决策状态
    DecisionState current_state = DecisionState::IDLE;
    DecisionState previous_state = DecisionState::IDLE;
    double last_state_change_time = 0.0;
    double last_decision_time = 0.0;

    NavigationState() = default;
  } state_;

  // 游戏状态
  GameState game_state_;
  mutable std::mutex game_state_mutex_;  // mutable 以便在const函数中使用

  // TF相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 导航动作客户端
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr current_goal_handle_;

  // 订阅器
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // 游戏状态订阅器
  rclcpp::Subscription<connection_layer::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<connection_layer::msg::RobotStatus>::SharedPtr robot_status_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr connection_timer_;
  rclcpp::TimerBase::SharedPtr decision_timer_;
  rclcpp::TimerBase::SharedPtr attack_check_timer_;

  // 互斥锁
  mutable std::mutex state_mutex_;
  mutable std::mutex map_mutex_;

  // 地图数据
  std::vector<int8_t> map_data_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_metadata_;

  // 初始化方法
  template <class T>
  void set_params(rclcpp::Node* node, const std::string& params_name, T& target, const T& default_val)
  {
    node->declare_parameter<T>(params_name, default_val);
    node->get_parameter(params_name, target);
  }
  void set_parameters();
  void calculate_enemy_base_position();
  void initialize_subscriptions();
  void initialize_timers();
  void initialize_navigation();

  // 回调函数
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void game_status_callback(const connection_layer::msg::GameStatus::SharedPtr msg);
  void robot_status_callback(const connection_layer::msg::RobotStatus::SharedPtr msg);

  // 定时器回调
  void update_status();
  void check_connection();
  void decision_tree_callback();
  void attack_check_callback();

  // 导航方法
  bool navigate_to_point_impl(double x, double y, double yaw = 0.0);
  bool navigate_to_pose(const geometry_msgs::msg::PoseStamped& pose);
  bool get_robot_position();

  // 导航回调
  void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
  void nav_feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                             const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void nav_result_callback(const GoalHandleNavigateToPose::WrappedResult& result);

  // 连接检查
  bool check_tf_connection();
  bool check_navigation_server();

  // 决策树方法
  void update_decision_state();
  void execute_decision_state();
  void change_state(DecisionState new_state);
  void handle_idle_state();
  void handle_waiting_state();
  void handle_capturing_center_state();
  void handle_returning_base_state();
  void handle_approaching_enemy_state();
  void handle_navigating_state();

  // 决策辅助方法
  double center_captured() const;
  bool is_low_hp() const;
  bool is_safe_hp() const;
  bool is_normal_hp() const;
  bool has_not_been_attacked_for(double duration) const;
  double get_hp_percentage() const;

  // 工具函数
  std::pair<double, double> calculate_approach_point() const;
  double calculate_distance(double x1, double y1, double x2, double y2) const;
  bool is_in_map_boundary(double x, double y) const;
  double get_current_time() const;

  // 日志辅助
  void log_info(const std::string& message) const;
  void log_warning(const std::string& message) const;
  void log_error(const std::string& message) const;
  void log_debug(const std::string& message) const;

  // 状态字符串转换
  std::string state_to_string(DecisionState state) const;
};

}  // namespace decision_tree

#endif  // DECISION_TREE_HPP
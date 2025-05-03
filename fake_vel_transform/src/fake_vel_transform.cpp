/**
 * @file fake_vel_transform.cpp
 * @brief 实现速度转换节点，用于处理机器人速度命令的坐标系转换和同步
 * @details 该节点主要用于：
 * 1. 处理机器人速度命令的坐标系转换
 * 2. 在导航系统中处理速度命令和局部路径规划的同步
 * 3. 支持额外的旋转速度叠加
 * 4. 维护虚拟坐标系和实际坐标系之间的变换关系
 */

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

// 用于判断速度是否为零的阈值
constexpr double EPSILON = 1e-5;
// 控制器超时时间（秒）
constexpr double CONTROLLER_TIMEOUT = 0.5;

/**
 * @brief 构造函数，初始化节点和参数
 * @param options 节点选项
 */
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // 声明和获取节点参数
  // robot_base_frame: 机器人基础坐标系名称
  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  // fake_robot_base_frame: 虚拟机器人基础坐标系名称
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  // odom_topic: 里程计话题名称
  this->declare_parameter<std::string>("odom_topic", "odom");
  // local_plan_topic: 局部路径规划话题名称
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  // cmd_spin_topic: 旋转速度命令话题名称
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  // input_cmd_vel_topic: 输入速度命令话题名称
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  // output_cmd_vel_topic: 输出速度命令话题名称
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  // init_spin_speed: 初始旋转速度
  this->declare_parameter<float>("init_spin_speed", 0.0);

  // 获取参数值
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);

  // 初始化TF广播器
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 创建速度命令发布器
  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  // 创建订阅器
  // 订阅旋转速度命令
  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  // 订阅速度命令
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  // 订阅里程计和局部路径规划消息
  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // 在 Navigation2 Humble 版本中，速度由控制器发布，没有时间戳。
  // 我们认为 velocity 与 local_plan 同时发布。
  // 因此，我们使用 ApproximateTime 策略来同步 'cmd_vel' 和 'odometry'。
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 创建50Hz定时器，用于发布TF变换
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

/**
 * @brief 处理旋转速度命令的回调函数
 * @param msg 旋转速度消息
 */
void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

/**
 * @brief 处理里程计消息的回调函数
 * @param msg 里程计消息
 */
void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // 注意：尚未与 local_plan 同步
  if ((rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

/**
 * @brief 处理速度命令的回调函数
 * @param msg 速度命令消息
 */
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  // 检查速度是否接近零
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (rclcpp::Clock().now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // 如果无法同步接收到的速度，直接发布转换后的速度
    auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } else {
    latest_cmd_vel_ = msg;
  }
}

/**
 * @brief 处理局部路径规划消息的回调函数
 * @param msg 局部路径规划消息
 */
void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // 更新控制器最后激活时间
  last_controller_activate_time_ = rclcpp::Clock().now();
}

/**
 * @brief 处理同步后的里程计和局部路径规划消息的回调函数
 * @param odom_msg 里程计消息
 * @param local_plan_msg 局部路径规划消息
 */
void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  // 更新机器人基础角度并转换速度
  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  float yaw_diff = current_robot_base_angle_;
  geometry_msgs::msg::Twist aft_tf_vel = transformVelocity(current_cmd_vel, yaw_diff);

  // 发布转换后的速度
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

/**
 * @brief 发布TF变换
 */
void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  // 设置旋转角度
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

/**
 * @brief 转换速度的核心函数
 * @param twist 输入速度
 * @param yaw_diff 偏航角差
 * @return 转换后的速度
 */
geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  // 添加额外旋转速度
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
  // 转换线速度分量
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)

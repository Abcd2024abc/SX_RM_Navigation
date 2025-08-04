#ifndef spin_conversion__spin_conversion_HPP_
#define spin_conversion__spin_conversion_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "spin_conversion/msg/spin_command.hpp"

namespace spin_conversion
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdSpinCallback(example_interfaces::msg::Float32::SharedPtr msg);
  void spinCommandCallback(const spin_conversion::msg::SpinCommand::SharedPtr msg);
  void publishTransform();
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<spin_conversion::msg::SpinCommand>::SharedPtr spin_command_sub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Path>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  float spin_speed_;

  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  double current_robot_base_angle_;
  rclcpp::Time last_controller_activate_time_;

  bool spin_enabled_;
  int8_t spin_direction_;
};

}  // namespace spin_conversion

#endif  // spin_conversion__spin_conversion_HPP_

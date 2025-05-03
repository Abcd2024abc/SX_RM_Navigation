#ifndef PATH_TRACKING_CONTROLLER__PATH_TRACKING_CONTROLLER_HPP_
#define PATH_TRACKING_CONTROLLER__PATH_TRACKING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "path_tracking_controller/pid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace path_tracking_controller
{

class PathTrackingController : public nav2_core::Controller
{
public:
  PathTrackingController() = default;
  ~PathTrackingController() override = default;

  // 生命周期管理
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // 核心功能
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // 路径处理
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  bool transformPose(
    const std::string & frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  // 前瞻点计算
  double getCostmapMaxExtent() const;
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);
  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist,
    const nav_msgs::msg::Path & transformed_plan);
  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    double r);

  // 参数管理
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // 速度控制
  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);
  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;
  void applyApproachVelocityScaling(const nav_msgs::msg::Path & path, double & linear_vel) const;
  bool isCollisionDetected(const nav_msgs::msg::Path & path);

private:
  // 曲率限制
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel);

  // 碰撞避让
  double calculateCollisionAvoidanceDirection(const nav_msgs::msg::Path & path) const;
  geometry_msgs::msg::TwistStamped createAvoidanceCommand(
    const std_msgs::msg::Header & header,
    double avoidance_angle) const;

  // 曲率计算
  double calculateCurvature(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist,
    double backward_dist) const;

  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point,
    const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  // 可视化
  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  // 路径处理辅助函数
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;
  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path,
    const std::vector<double> & cumulative_distances,
    double target_distance) const;

  // 零速度命令生成
  geometry_msgs::msg::TwistStamped createZeroVelocityCommand(
    const std_msgs::msg::Header & header) const;

  /**
   * @brief 检测到碰撞后尝试相反方向移动并动态检测是否脱离碰撞代价
   * @param pose 当前位姿
   * @param velocity 当前速度
   * @param transformed_plan 转换后的路径
   * @return 是否成功脱离碰撞
   */
  bool attemptCollisionRecovery(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    const nav_msgs::msg::Path & transformed_plan);

private:
  // ROS2 相关成员
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("PathTrackingController")};
  rclcpp::Clock::SharedPtr clock_;

  // 控制器相关成员
  std::shared_ptr<PID> move_pid_;
  std::shared_ptr<PID> heading_pid_;
  double last_velocity_scaling_factor_{0.0};

  // 参数
  double translation_kp_{0.0}, translation_ki_{0.0}, translation_kd_{0.0};
  bool enable_rotation_{true};
  double rotation_kp_{0.0}, rotation_ki_{0.0}, rotation_kd_{0.0};
  double min_max_sum_error_{0.0};
  double control_duration_{0.0};
  double max_robot_pose_search_dist_{0.0};
  bool use_interpolation_{true};
  double lookahead_dist_{0.0};
  bool use_velocity_scaled_lookahead_dist_{true};
  double min_lookahead_dist_{0.0};
  double max_lookahead_dist_{0.0};
  double lookahead_time_{0.0};
  bool use_rotate_to_heading_{true};
  double use_rotate_to_heading_treshold_{0.0};
  double v_linear_min_{0.0}, v_linear_max_{0.0};
  double v_angular_min_{0.0}, v_angular_max_{0.0};
  double min_approach_linear_velocity_{0.0};
  double approach_velocity_scaling_dist_{0.0};
  double curvature_min_{0.0}, curvature_max_{0.0};
  double reduction_ratio_at_high_curvature_{0.0};
  double curvature_forward_dist_{0.0};
  double curvature_backward_dist_{0.0};
  double max_velocity_scaling_factor_rate_{0.0};
  tf2::Duration transform_tolerance_;

  // 路径和发布者
  nav_msgs::msg::Path global_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr curvature_points_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  // 发布速度命令的发布者

  // 线程安全
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace path_tracking_controller

#endif  // PATH_TRACKING_CONTROLLER__PATH_TRACKING_CONTROLLER_HPP_

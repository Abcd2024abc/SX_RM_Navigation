#ifndef map_cost_extend__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define map_cost_extend__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::BackUpFreeSpace
 * @brief 一个增强的 back_up 行为，能够朝向自由空间移动
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  /**
   * @brief 构造函数
   */
  BackUpFreeSpace() = default;

  /**
   * @brief 禁用拷贝构造和赋值
   */
  BackUpFreeSpace(const BackUpFreeSpace &) = delete;
  BackUpFreeSpace & operator=(const BackUpFreeSpace &) = delete;

  /**
   * @brief 配置行为参数
   */
  void onConfigure() override;

  /**
   * @brief 清理行为资源
   */
  void onCleanup() override;

  /**
   * @brief 初始化行为运行
   * @param command 要执行的目标
   * @return 行为的状态
   */
  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

  /**
   * @brief 循环更新行为
   * @return 行为的状态
   */
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  using Costmap = nav2_msgs::msg::Costmap;
  using Pose2D = geometry_msgs::msg::Pose2D;

  /**
   * @brief 在代价地图中收集自由空间点
   * @param costmap 代价地图
   * @param pose 当前位姿
   * @param radius 搜索半径
   * @return 自由空间点集合
   * @throw std::runtime_error 当无法访问代价地图数据时
   */
  std::vector<geometry_msgs::msg::Point> gatherFreePoints(
    const Costmap & costmap,
    const Pose2D & pose,
    float radius);

  /**
   * @brief 查找最佳移动方向
   * @param costmap 代价地图
   * @param pose 当前位姿
   * @param start_angle 起始角度
   * @param end_angle 结束角度
   * @param radius 搜索半径
   * @param angle_increment 角度增量
   * @return 最佳移动方向(弧度)
   */
  float findBestDirection(
    const Costmap & costmap,
    const Pose2D & pose,
    float start_angle,
    float end_angle,
    float radius,
    float angle_increment);

  /**
   * @brief 检查路径是否无碰撞
   * @param distance 当前移动距离
   * @param cmd_vel 速度指令
   * @param pose 当前位姿
   * @return 是否安全无碰撞
   */
  bool isCollisionFree(float distance, geometry_msgs::msg::Twist* cmd_vel, const Pose2D& pose);

  /**
   * @brief 检查点是否安全
   * @param costmap 代价地图
   * @param x x坐标
   * @param y y坐标
   * @return 点是否安全
   */
  bool isPointSafe(const Costmap& costmap, float x, float y);

  /**
   * @brief 可视化自由点和方向。
   *
   * 此函数用于可视化自由点和方向。
   *
   * @param pose 当前的位姿。
   * @param radius 搜索半径。
   * @param first_safe_angle 第一个安全角度。
   * @param last_unsafe_angle 最后一个不安全角度。
   */
  void visualize(
    Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle);

private:
  // 新增辅助方法声明
  bool waitForCostmapService();
  std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> getCostmap();
  bool updateRobotPose(geometry_msgs::msg::PoseStamped & pose);
  Pose2D convertToPose2D(const geometry_msgs::msg::PoseStamped & pose);
  void setMotionParameters(const std::shared_ptr<const BackUpAction::Goal> command, float angle);

  // 成员变量
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    marker_pub_;
  double twist_x_{0.0};
  double twist_y_{0.0};
  std::string service_name_;
  double max_radius_{1.0};
  bool visualize_{false};
};

}  // namespace pb_nav2_behaviors

#endif  // map_cost_extend__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#ifndef NAV_DECISION_MAKER_HPP
#define NAV_DECISION_MAKER_HPP

#include <memory>
#include <random>
#include <cmath>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_decision_maker/srv/set_target.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigationActionClient = rclcpp_action::Client<NavigateToPose>;
using NavigationActionGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using SetTarget = nav_decision_maker::srv::SetTarget;

class NavDecisionMaker : public rclcpp::Node {
public:
    NavDecisionMaker();
    void retry_connection();
    void handle_set_target(const std::shared_ptr<SetTarget::Request> request, std::shared_ptr<SetTarget::Response> response);
    void generate_dodge_goal();
    void send_goal();

private:
    double target_x;
    double target_y;
    bool is_active_;
    bool is_navigating_;
    bool enable_dodge_;
    bool is_dodging_;
    double dodge_radius_MIN;
    double dodge_radius_MAX;
    NavigationActionClient::SharedPtr action_client_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr dodge_timer_;
    rclcpp::Service<SetTarget>::SharedPtr set_target_service_;
    std::mt19937 rng_;
};

#endif // NAV_DECISION_MAKER_HPP

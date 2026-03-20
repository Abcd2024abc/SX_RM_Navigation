#include <memory>
#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

using namespace std::chrono_literals;

class TFTransform : public rclcpp::Node
{
public:
  TFTransform() : Node("tf_transform")
  {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->StaticTF();
    timer_ = create_wall_timer(10ms, std::bind(&TFTransform::DynamicTF, this));
  }

  void StaticTF()
  {
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "gimbal_yaw";
    static_transform.child_frame_id = "mid360";

    static_transform.transform.translation.x = 0.18;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(-M_PI / 4, 0.0, -M_PI / 2);
    static_transform.transform.rotation = tf2::toMsg(q);

    broadcaster_->sendTransform(static_transform);
  }

  void DynamicTF()
  {
    geometry_msgs::msg::TransformStamped dynamic_transform;
    dynamic_transform.header.stamp = this->now();
    dynamic_transform.header.frame_id = "base_footprint";
    dynamic_transform.child_frame_id = "gimbal";
    dynamic_transform.transform.translation.x = 0.0;
    dynamic_transform.transform.translation.y = 0.0;
    dynamic_transform.transform.translation.z = 0.28;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    dynamic_transform.transform.rotation = tf2::toMsg(q);

    broadcaster_->sendTransform(dynamic_transform);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFTransform>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
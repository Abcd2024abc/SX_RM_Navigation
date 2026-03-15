#pragma once

#include <memory>
#include <thread>
#include <vector>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "connection_layer/msg/robot_status.hpp"
#include "connection_layer/msg/game_status.hpp"

namespace connection_layer
{

#define serial drivers::serial_driver

constexpr uint8_t SOF_RECEIVE = 0xAA;
constexpr uint8_t SOF_SEND = 0xAA;

class ConnectionLayer : public rclcpp::Node
{
public:
  ConnectionLayer(const rclcpp::NodeOptions& options);

  ~ConnectionLayer();

private:
  // 校验函数
  bool openSerial();

  // 接收函数
  void receive_data();

  // 发送函数
  void send_data();

  // 底盘速度命令回调
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // 工具函数
  void pack_float(uint8_t* buf, float f);

  float speed_vx, speed_vy;

  std::atomic<bool> stop_;
  std::thread receive_thread_;
  std::thread send_thread_;

  std::string port_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  std::unique_ptr<serial::SerialDriver> serial_driver_;
  std::shared_ptr<serial::SerialPortConfig> serial_config_;

  rclcpp::Publisher<connection_layer::msg::RobotStatus>::SharedPtr robot_pub_;
  rclcpp::Publisher<connection_layer::msg::GameStatus>::SharedPtr game_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};
}  // namespace connection_layer
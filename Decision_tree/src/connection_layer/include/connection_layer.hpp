#ifndef connection_layer__connection_layer_HPP_
#define connection_layer__connection_layer_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "serial_driver/serial_driver.hpp"
#include "connection_layer/msg/game_status.hpp"
#include "connection_layer/msg/robot_status.hpp"
#include "connection_layer/msg/rfid_status.hpp"

#include "data_packet.hpp"

namespace connection_layer
{
    class ConnectionLayer : public rclcpp::Node
    {
    public:
        ConnectionLayer(const rclcpp::NodeOptions &options);
        ~ConnectionLayer() override;

    private:
        // ============ 串口与连接管理 ============
        std::atomic<bool> is_usb_ok_{false};                                      ///< 串口连接状态标志
        std::string device_name_;                                                 ///< 串口设备路径（如 /dev/ttyUSB0）
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_; ///< 串口配置（波特率、校验等）
        std::unique_ptr<IoContext> owned_ctx_;                                    ///< ASIO IO context（管理异步操作）
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;     ///< 串口驱动对象
        std::mutex serial_mutex_;                                                ///< 串口状态标志互斥锁

        // ============ 后台工作线程 ============
        std::thread receive_thread_;             ///< 串口接收线程
        std::thread send_thread_;                ///< 串口发送线程
        std::thread serial_port_protect_thread_; ///< 串口保护线程

        // ============ 发布者（下位机数据 -> ROS Topic） ============
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;                       ///< IMU 话题（惯性、角速度）
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_motion_pub_;          ///< 机器人运动状态（速度向量）
        rclcpp::Publisher<connection_layer::msg::RfidStatus>::SharedPtr rfid_status_pub_;   ///< 增益点占领状态
        rclcpp::Publisher<connection_layer::msg::RobotStatus>::SharedPtr robot_status_pub_; ///< 本机器人状态（HP、热量、位置等）
        rclcpp::Publisher<connection_layer::msg::GameStatus>::SharedPtr game_status_pub_;  ///< 比赛进度

        // ============ 订阅者（ROS Topic -> 下位机数据） ============

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; ///< 底盘速度命令（导航或操作）

        SendRobotCmdData send_robot_cmd_data_;

        void getParams();                                      // 获取配置
        void createPublisher();                                // 创建发布者
        void createSubscription();                             // 创建订阅者
        void serialPortProtect();                              // 串口保护线程函数
        void receiveData();                                    // 串口接收线程函数
        void sendData();                                       // 串口发送线程函数
        void publishImuData(ReceiveImuData &data);             // 发布IMU数据
        void publishRobotMotion(ReceiveRobotMotionData &data); // 发布机器人运动数据
        void publishRfidStatus(ReceiveRfidStatus &data);       // 发布增益点状态
        void publishRobotStatus(ReceiveRobotStatus &data);     // 发布机器人状态
        void publishGameStatus(ReceiveGameStatus &data);       // 发布比赛状态

        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg); // 底盘速度命令回调

        float last_hp_; ///< 上一次收到的 HP 值（用于检测血量变化）
    };
} // namespace connection_layer

#endif
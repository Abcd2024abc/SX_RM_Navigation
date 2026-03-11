#include "connection_layer.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   ///< 串口连接失败时的重试等待时间（ms）
#define USB_PROTECT_SLEEP_TIME 1000  ///< 串口保护线程的循环检测周期（ms）

using namespace std::chrono_literals;

namespace connection_layer
{
ConnectionLayer::ConnectionLayer(const rclcpp::NodeOptions& options)
  : Node("connection_layer", options)
  , owned_ctx_{ new IoContext(2) }
  ,  // 创建 IO context，2 个工作线程用于串口异步操作
  serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }  // 创建串口驱动
{
  {
    RCLCPP_INFO(get_logger(), "启动连接层");

    // 获取参数
    getParams();

    // 创建发布者
    createPublisher();

    // 创建订阅者
    createSubscription();

    // 启动三个后台线程
    serial_port_protect_thread_ = std::thread(&ConnectionLayer::serialPortProtect, this);
    receive_thread_ = std::thread(&ConnectionLayer::receiveData, this);
    send_thread_ = std::thread(&ConnectionLayer::sendData, this);
  }
}

ConnectionLayer::~ConnectionLayer()
{
  if (send_thread_.joinable())
  {
    send_thread_.join();
  }

  if (receive_thread_.joinable())
  {
    receive_thread_.join();
  }

  if (serial_port_protect_thread_.joinable())
  {
    serial_port_protect_thread_.join();
  }

  if (serial_driver_->port()->is_open())
  {
    serial_driver_->port()->close();
  }

  if (owned_ctx_)
  {
    owned_ctx_->waitForExit();
  }
};

void ConnectionLayer::getParams()
// 获取配置
{
  // 引入串口驱动相关的枚举类型
  using FlowControl = drivers::serial_driver::FlowControl;  // 流量控制类型
  using Parity = drivers::serial_driver::Parity;            // 奇偶校验类型
  using StopBits = drivers::serial_driver::StopBits;        // 停止位类型

  uint32_t baud_rate{};         // 波特率变量，初始化为0
  auto fc = FlowControl::NONE;  // 流量控制：默认无流控
  auto pt = Parity::NONE;       // 奇偶校验：默认无校验
  auto sb = StopBits::ONE;      // 停止位：默认1个停止位

  // 读取设备名称参数
  try
  {
    device_name_ = declare_parameter<std::string>("device_name", "");
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "提供的设备名称无效");
    throw ex;
  }

  // 读取波特率参数
  try
  {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "提供的波特率无效");
    throw ex;
  }

  // 读取流控制方式参数
  try
  {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    // 根据字符串值转换为对应的枚举类型
    if (fc_string == "none")
    {
      fc = FlowControl::NONE;  // 无流控
    }
    else if (fc_string == "hardware")
    {
      fc = FlowControl::HARDWARE;  // 硬件流控
    }
    else if (fc_string == "software")
    {
      fc = FlowControl::SOFTWARE;  // 软件流控
    }
    else
    {
      throw std::invalid_argument{ "流控制参数必须是以下之一：无、软件或硬件" };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "提供的流控制无效");
    throw ex;
  }

  // 读取校验方式参数
  try
  {
    const auto pt_string = declare_parameter<std::string>("parity", "");  // 读取奇偶校验字符串参数

    // 根据字符串值转换为对应的枚举类型
    if (pt_string == "none")
    {
      pt = Parity::NONE;  // 无校验
    }
    else if (pt_string == "odd")
    {
      pt = Parity::ODD;  // 奇校验
    }
    else if (pt_string == "even")
    {
      pt = Parity::EVEN;  // 偶校验
    }
    else
    {
      throw std::invalid_argument{ "奇偶校验参数必须是以下之一：无、奇数或偶数" };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "提供的奇偶校验无效");
    throw ex;
  }

  // 读取停止位参数
  try
  {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");  // 读取停止位字符串参数

    // 根据字符串值转换为对应的枚举类型
    if (sb_string == "1" || sb_string == "1.0")
    {
      sb = StopBits::ONE;  // 1个停止位
    }
    else if (sb_string == "1.5")
    {
      sb = StopBits::ONE_POINT_FIVE;  // 1.5个停止位
    }
    else if (sb_string == "2" || sb_string == "2.0")
    {
      sb = StopBits::TWO;  // 2个停止位
    }
    else
    {
      throw std::invalid_argument{ "停止位参数必须是以下之一：1、1.5或2。" };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "提供的停止位无效");
    throw ex;
  }

  // 创建串口配置对象，使用上面读取的参数
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
};

void ConnectionLayer::createPublisher()
// 创建发布者
{
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);

  rfid_status_pub_ = this->create_publisher<connection_layer::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ = this->create_publisher<connection_layer::msg::RobotStatus>("referee/robot_status", 10);
  game_status_pub_ = this->create_publisher<connection_layer::msg::GameStatus>("referee/game_status", 10);
};

void ConnectionLayer::createSubscription()
// 创建订阅者
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ConnectionLayer::cmdVelCallback, this, std::placeholders::_1));
};

void ConnectionLayer::serialPortProtect()
// 串口保护线程函数
{
  RCLCPP_INFO(get_logger(), "启动串口保护线程");

  // 初始化串口配置
  serial_driver_->init_port(device_name_, *device_config_);

  // 尝试打开串口
  try
  {
    if (!serial_driver_->port()->is_open())
    {
      serial_driver_->port()->open();
      RCLCPP_INFO(get_logger(), "Serial port opened!");
      is_usb_ok_ = true;
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "打开串口失败 : %s", ex.what());
    is_usb_ok_ = false;
  }

  // 主循环：持续监控串口状态
  while (rclcpp::ok())
  {
    if (!is_usb_ok_)
    {
      // 串口状态异常，尝试重新连接
      try
      {
        if (serial_driver_->port()->is_open())
        {
          serial_driver_->port()->close();
        }

        // 尝试打开串口
        serial_driver_->port()->open();

        if (serial_driver_->port()->is_open())
        {
          RCLCPP_INFO(get_logger(), "串口已打开!");
          is_usb_ok_ = true;
        }
      }
      catch (const std::exception& ex)
      {
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "打开串口失败: %s", ex.what());
      }
    }

    // 定期检查（避免过度占用 CPU）
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }
};

void ConnectionLayer::receiveData()
// 串口接收线程函数
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");  // 日志：开始接收数据

  std::vector<uint8_t> sof(1);        // 创建大小为1的向量，用于存储帧起始字节
  std::vector<uint8_t> receive_data;  // 存储完整接收数据的向量

  int sof_count = 0;    // SOF计数器：记录连续找到的SOF字节数量
  int retry_count = 0;  // 重试计数器：记录串口连接失败重试次数

  while (rclcpp::ok())
  {  // 主循环：持续运行直到ROS2节点关闭
    if (!is_usb_ok_)
    {  // 检查串口连接状态
      RCLCPP_WARN(get_logger(), "接收USB异常常！重试次数: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;  // 串口异常，跳过本次循环继续等待
    }

    try
    {
      serial_driver_->port()->receive(sof);

      // 检查是否为期望的帧起始字节
      if (sof[0] != SOF_RECEIVE)
      {
        sof_count++;
        RCLCPP_INFO(get_logger(), "未预期的SOF, cnt=%d", sof_count);
        continue;
      }

      sof_count = 0;  // 重置SOF计数器

      // SOF_RECEIVE验证通过后，读取帧头剩余部分
      std::vector<uint8_t> header_frame_buf(3);  // 创建缓冲区存储帧头剩余3个字节（不包括已读取的SOF）

      serial_driver_->port()->receive(header_frame_buf);          // 读取帧头剩余部分
      header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 将已读取的SOF插入到缓冲区开头

      HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);  // 将字节流转换为HeaderFrame结构体

      // 根据帧头中的长度字段读取数据段
      std::vector<uint8_t> data_buf(header_frame.len);               // 创建缓冲区：数据长度字节
      int received_len = serial_driver_->port()->receive(data_buf);  // 读取数据段
      int received_len_sum = received_len;                           // 记录已读取的总字节数

      // 检查数据是否完整接收（考虑一次性读取可能不完整的情况）
      int remain_len = header_frame.len - received_len;  // 计算剩余未读取的数据长度

      if (received_len <= 0)
      {
        RCLCPP_WARN(get_logger(), "收到无效的数据长度: %d", received_len);
        continue;
      }

      while (remain_len > 0)
      {                                                              // 循环读取剩余数据直到完整接收
        std::vector<uint8_t> remain_buf(remain_len);                 // 创建剩余数据缓冲区
        received_len = serial_driver_->port()->receive(remain_buf);  // 读取剩余数据
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        received_len_sum += received_len;  // 更新已读取总字节数
        remain_len -= received_len;        // 更新剩余数据长度
      }

      // 将帧头字节流添加到数据缓冲区开头，形成完整数据包
      data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

      // 根据数据包ID分发处理
      switch (header_frame.id)
      {
        case ID_IMU: {  // IMU数据
          ReceiveImuData imu_data = fromVector<ReceiveImuData>(data_buf);
          publishImuData(imu_data);  // 发布IMU数据
        }
        break;
        case ID_ROBOT_MOTION: {  // 机器人运动数据
          ReceiveRobotMotionData robot_motion_data = fromVector<ReceiveRobotMotionData>(data_buf);
          publishRobotMotion(robot_motion_data);  // 发布机器人运动数据
        }
        break;
        case ID_RFID_STATUS: {  // RFID状态数据
          ReceiveRfidStatus rfid_status_data = fromVector<ReceiveRfidStatus>(data_buf);
          publishRfidStatus(rfid_status_data);  // 发布RFID状态
        }
        break;
        case ID_ROBOT_STATUS: {  // 机器人状态数据
          ReceiveRobotStatus robot_status_data = fromVector<ReceiveRobotStatus>(data_buf);
          publishRobotStatus(robot_status_data);  // 发布机器人状态
        }
        break;
        default: {  // 未知数据包ID
          RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
        }
        break;
      }
    }
    catch (const std::exception& ex)
    {  // 捕获异常
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;  // 标记串口连接异常
    }
  }
};

void ConnectionLayer::sendData()
// 串口发送线程函数
{
  RCLCPP_INFO(get_logger(), "开始发送数据");

  // 设置数据帧头部信息
  send_robot_cmd_data_.frame_header.sof = SOF_SEND;                      // 帧起始标志
  send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;                   // 命令ID：机器人控制
  send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 3;  // 数据长度（不包括帧头）;
  send_robot_cmd_data_.data.speed_vector.vx = 0;                         // X轴方向速度，单位为m/s
  send_robot_cmd_data_.data.speed_vector.vy = 0;                         // Y轴方向速度，单位为m/s
  // 绕Z轴旋转角速度，单位为rad/s，未启用。
  // send_robot_cmd_data_.data.speed_vector.wz = 0;

  // 主发送循环：持续发送控制指令直到ROS2关闭
  while (rclcpp::ok())
  {
    // 检查USB串口连接状态
    if (!is_usb_ok_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try
    {
      // 将数据结构转换为字节流
      std::vector<uint8_t> send_data = toVector(send_robot_cmd_data_);

      // 通过串口发送数据给机器人控制器
      serial_driver_->port()->send(send_data);
    }
    catch (const std::exception& ex)
    {
      is_usb_ok_ = false;
    }

    // 控制发送频率：每5ms发送一次（200Hz）
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
};

void ConnectionLayer::publishImuData(ReceiveImuData& imu_data)
// 发布IMU数据
{
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.header.stamp = now();
  imu_msg.header.frame_id = "imu_frame";

  tf2::Quaternion q;
  q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
  imu_msg.orientation = tf2::toMsg(q);

  imu_msg.angular_velocity.x = imu_data.data.roll_vel;
  imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
  imu_msg.angular_velocity.z = imu_data.data.yaw_vel;

  imu_pub_->publish(imu_msg);
};

void ConnectionLayer::publishRobotMotion(ReceiveRobotMotionData& robot_motion)
// 发布机器人运动数据
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = robot_motion.data.speed_vector.vx;
  msg.linear.y = robot_motion.data.speed_vector.vy;
  msg.angular.z = robot_motion.data.speed_vector.wz;

  robot_motion_pub_->publish(msg);
};

void ConnectionLayer::publishRfidStatus(ReceiveRfidStatus& rfid_status)
// 发布增益点状态
{
  connection_layer::msg::RfidStatus msg;

  msg.center_gain_point = rfid_status.data.center_gain_point;

  rfid_status_pub_->publish(msg);
};

void ConnectionLayer::publishRobotStatus(ReceiveRobotStatus& robot_status)
// 发布机器人状态
{
  connection_layer::msg::RobotStatus msg;

  // 基本信息
  msg.robot_id = robot_status.data.robot_id;  // 机器人ID

  // 生命值相关
  msg.current_hp = robot_status.data.current_hp;  // 当前生命值
  msg.maximum_hp = robot_status.data.maximum_hp;  // 最大生命值

  robot_status_pub_->publish(msg);
};

void ConnectionLayer::publishGameStatus(ReceiveGameStatus& game_status)
// 发布比赛状态
{
  connection_layer::msg::GameStatus msg;

  msg.game_progress = game_status.data.game_progress;          // 比赛阶段
  msg.stage_remain_time = game_status.data.stage_remain_time;  // 剩余时间

  game_status_pub_->publish(msg);
};

void ConnectionLayer::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
// 底盘速度命令回调;
{
  send_robot_cmd_data_.data.speed_vector.vx = msg->linear.x;
  send_robot_cmd_data_.data.speed_vector.vy = msg->linear.y;
  // send_robot_cmd_data_.data.speed_vector.wz = msg->angular.z;
}

}  // namespace connection_layer
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(connection_layer::ConnectionLayer)
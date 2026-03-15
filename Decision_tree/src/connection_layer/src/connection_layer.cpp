#include "connection_layer.hpp"

namespace connection_layer
{
ConnectionLayer::ConnectionLayer(const rclcpp::NodeOptions& options) : Node("serial_bridge_node", options)
{
  // 初始化
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<std::string>("flow_control", "none");
  this->declare_parameter<std::string>("parity", "none");
  this->declare_parameter<std::string>("stop_bits", "1");

  port_ = this->get_parameter("port").as_string();
  int baudrate = this->get_parameter("baudrate").as_int();

  auto fc = serial::FlowControl::NONE;  // 流量控制：默认无流控
  auto pt = serial::Parity::NONE;       // 奇偶校验：默认无校验
  auto sb = serial::StopBits::ONE;      // 停止位：默认1个停止位
  serial::SerialPortConfig config(baudrate, fc, pt, sb);

  io_context_ = std::make_shared<drivers::common::IoContext>(2);
  serial_driver_ = std::make_unique<serial::SerialDriver>(*io_context_);

  serial_config_ = std::make_shared<serial::SerialPortConfig>(config);

  // Create Publisher
  robot_pub_ = this->create_publisher<connection_layer::msg::RobotStatus>("robot/status", 10);
  game_pub_ = this->create_publisher<connection_layer::msg::GameStatus>("game/status", 10);

  // Create Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ConnectionLayer::cmdVelCallback, this, std::placeholders::_1));

  receive_thread_ = std::thread(&ConnectionLayer::receive_data, this);
  send_thread_ = std::thread(&ConnectionLayer::send_data, this);
}

ConnectionLayer::~ConnectionLayer()
{
  stop_ = true;
  if (serial_driver_->port() && serial_driver_->port()->is_open())
    serial_driver_->port()->close();
  if (receive_thread_.joinable())
    receive_thread_.join();
  if (send_thread_.joinable())
    send_thread_.join();
}

bool ConnectionLayer::openSerial()
{
  try
  {
    if (serial_driver_->port()->is_open())
      serial_driver_->port()->close();

    serial_driver_->init_port(port_, *serial_config_);
    serial_driver_->port()->open();
    RCLCPP_INFO(this->get_logger(), "Open the serial port: %s", port_.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    return false;
  }
}

void ConnectionLayer::receive_data()
{
  try
  {
    while (rclcpp::ok() && !stop_)
    {
      if (!openSerial())
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      int sof_count = 0;

      while (rclcpp::ok() && !stop_)
      {
        std::vector<uint8_t> sof(2);
        serial_driver_->port()->receive(sof);

        if (sof[0] != SOF_RECEIVE)
        {
          sof_count++;
          continue;
        }

        sof_count = 0;

        std::vector<uint8_t> data_field(2);
        serial_driver_->port()->receive(data_field);

        uint16_t data_len = (static_cast<uint16_t>(data_field[1]) << 8) | data_field[0];

        std::vector<uint8_t> buffer(data_len);

        int received_len = serial_driver_->port()->receive(buffer);
        int received_len_sum = received_len;

        int remain_len = data_len - received_len;

        if (received_len <= 0)
        {
          RCLCPP_WARN(get_logger(), "Invalid data length: %d", received_len);
          continue;
        }

        while (remain_len > 0)
        {
          std::vector<uint8_t> remain_buf(remain_len);                 // 创建剩余数据缓冲区
          received_len = serial_driver_->port()->receive(remain_buf);  // 读取剩余数据
          buffer.insert(buffer.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
          received_len_sum += received_len;  // 更新已读取总字节数
          remain_len -= received_len;        // 更新剩余数据长度
        }

        if (received_len_sum != data_len)
        {
          RCLCPP_WARN(get_logger(), "Data length mismatch: expected %d, received %d", data_len, received_len_sum);
          continue;
        }

        std::vector<uint8_t> checksum(1);
        serial_driver_->port()->receive(checksum);

        uint8_t calc_checksum = 0;

        // 累加
        calc_checksum += sof[0];
        calc_checksum += sof[1];
        calc_checksum += data_field[0];
        calc_checksum += data_field[1];
        for (uint8_t byte : buffer)
        {
          calc_checksum += byte;
        }

        // 校验和验证
        if (calc_checksum != checksum[0])
        {
          RCLCPP_ERROR(get_logger(), "Checksum mismatch! Received:0x%02X, Calculated:0x%02X", checksum[0],
                       calc_checksum);
          continue;
        }

        // 解析数据区
        uint8_t* data_ptr = buffer.data();

        auto read_float = [](uint8_t*& ptr) -> float {
          uint32_t u = (uint32_t(ptr[3]) << 24) | (uint32_t(ptr[2]) << 16) | (uint32_t(ptr[1]) << 8) | ptr[0];
          float f;
          memcpy(&f, &u, 4);
          ptr += 4;
          return f;
        };

        float fr = read_float(data_ptr);
        float fl = read_float(data_ptr);
        float rr = read_float(data_ptr);
        float rl = read_float(data_ptr);
        float hp = read_float(data_ptr);
        float gain = read_float(data_ptr);
        float progress = read_float(data_ptr);
        float remaining = read_float(data_ptr);

        // 发布消息
        connection_layer::msg::RobotStatus robot_msg;
        robot_msg.fr = fr;
        robot_msg.fl = fl;
        robot_msg.rr = rr;
        robot_msg.rl = rl;
        robot_msg.hp = hp;
        robot_pub_->publish(robot_msg);

        connection_layer::msg::GameStatus game_msg;
        game_msg.gain = gain;
        game_msg.progress = progress;
        game_msg.remaining = remaining;
        game_pub_->publish(game_msg);
      }
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
  }
}

void ConnectionLayer::send_data()
{
  std::vector<uint8_t> tx_buffer(32);
  uint16_t idx = 0;

  tx_buffer[idx++] = 0xAA;
  tx_buffer[idx++] = 0x55;

  // 长度占位
  uint16_t len_field_pos = idx;
  idx += 2;

  // 数据区起始
  uint16_t data_start_idx = idx;

  // 数据区
  pack_float(&tx_buffer[idx], speed_vx);
  idx += 4;
  pack_float(&tx_buffer[idx], speed_vy);
  idx += 4;

  uint16_t data_len = idx - data_start_idx;

  // 回填长度字段
  tx_buffer[len_field_pos] = (uint8_t)(data_len & 0xFF);
  tx_buffer[len_field_pos + 1] = (uint8_t)((data_len >> 8) & 0xFF);

  // 计算校验和（累加和，取低字节）
  uint8_t checksum = 0;
  for (uint16_t i = 0; i < idx; i++)
  {
    checksum += tx_buffer[i];
  }
  tx_buffer[idx++] = checksum;

  // 通过串口发送
  serial_driver_->port()->send(tx_buffer);
}

void ConnectionLayer::pack_float(uint8_t* buf, float f)
{
  uint32_t u;
  memcpy(&u, &f, 4);
  buf[0] = (uint8_t)(u & 0xFF);
  buf[1] = (uint8_t)((u >> 8) & 0xFF);
  buf[2] = (uint8_t)((u >> 16) & 0xFF);
  buf[3] = (uint8_t)((u >> 24) & 0xFF);
}

void ConnectionLayer::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_vx = msg->linear.x;
  speed_vy = msg->linear.y;
  // speed_wz = msg->angular.z;
}
}  // namespace connection_layer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(connection_layer::ConnectionLayer)
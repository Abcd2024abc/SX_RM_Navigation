#ifndef CONNECTION_LAYER__DATA_PACKET_HPP_
#define CONNECTION_LAYER__DATA_PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace connection_layer
{
    const uint8_t SOF_RECEIVE = 0x5A;
    const uint8_t SOF_SEND = 0x5A;

    const uint8_t ID_IMU = 0x01;          ///< IMU（惯性测量单元）数据包
    const uint8_t ID_ROBOT_MOTION = 0x02; ///< 机器人运动状态包（速度、位置）
    const uint8_t ID_RFID_STATUS = 0x03;  ///< RFID 区域获得点状态包
    const uint8_t ID_ROBOT_STATUS = 0x04; ///< 当前机器人状态包

    const uint8_t ID_ROBOT_CMD = 0x01; ///< 机器人控制命令包
    /**
     * @brief 数据帧头结构
     *
     * 用于标识和校验每个串口数据包的头部信息。
     * 帧头采用紧凑排列（packed）以保证与下位机二进制协议兼容
     */
    struct HeaderFrame
    {
        uint8_t sof;
        uint8_t len;
        uint8_t id;
    } __attribute__((packed));

    struct ReceiveImuData
    {
        HeaderFrame frame_header;
        uint32_t time_stamp;

        struct
        {
            float yaw;       ///< 偏航角（rad）
            float pitch;     ///< 俯仰角（rad）
            float roll;      ///< 滚转角（rad）
            float yaw_vel;   ///< 偏航角速度（rad/s）
            float pitch_vel; ///< 俯仰角速度（rad/s）
            float roll_vel;  ///< 滚转角速度（rad/s）
            // 注：以下字段未启用，待定。
            // float x_accel;  // 加速度 X (m/s^2)
            // float y_accel;  // 加速度 Y (m/s^2)
            // float z_accel;  // 加速度 Z (m/s^2)
        } __attribute__((packed)) data;
    } __attribute__((packed));

    struct ReceiveRobotMotionData
    {
        HeaderFrame frame_header;
        uint32_t time_stamp;

        struct
        {
            struct
            {
                float vx; ///< X 方向线速度（m/s）
                float vy; ///< Y 方向线速度（m/s）
                float wz; ///< Z 轴旋转角速度（rad/s）
            } __attribute__((packed)) speed_vector;
        } __attribute__((packed)) data;
    } __attribute__((packed));

    struct ReceiveRfidStatus
    {
        HeaderFrame frame_header;
        uint32_t time_stamp;

        struct
        {
            uint32_t center_gain_point; ///< 中央增益点获得状态
        } __attribute__((packed)) data;
    } __attribute__((packed));

    struct ReceiveRobotStatus
    {
        HeaderFrame frame_header;
        uint32_t time_stamp;

        struct
        {
            uint8_t robot_id;    ///< 本机器人 ID
            uint16_t current_hp; ///< 当前血量（HP）
            uint16_t maximum_hp; ///< 最大血量
        } __attribute__((packed)) data;
    } __attribute__((packed));

    struct ReceiveGameStatus
    {
        HeaderFrame frame_header;
        uint32_t time_stamp;

        struct
        {
            uint8_t game_progress;      ///< 比赛进度(0=未开始，1=进行中，2=结束）
            uint16_t stage_remain_time; ///< 剩余时间（秒）
        } __attribute__((packed)) data;
    } __attribute__((packed));

    struct SendRobotCmdData
    {
        HeaderFrame frame_header;
        uint32_t time_stamp;

        struct
        {
            struct
            {
                float vx; ///< X 方向线速度（m/s，前进为正）
                float vy; ///< Y 方向线速度（m/s，左为正）
                ///< Z 轴旋转角速度，小陀螺控制，未启用
                // float wz;
            } __attribute__((packed)) speed_vector;
        } __attribute__((packed)) data;
    } __attribute__((packed));

    /**
     * @brief 从字节向量解析为结构体
     * @return T 解析后的结构体对象
     * @note 结构体必须使用 __attribute__((packed)) 以确保内存布局与二进制协议一致
     */
    template <typename T>
    inline T fromVector(const std::vector<uint8_t> &data)
    {
        T packet;
        // 将字节数组内容复制到结构体内存
        std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
        return packet;
    }

    /**
     * @brief 从结构体转换为字节向量
     * @return std::vector<uint8_t> 转换后的字节向量（可用于串口发送）
     * @note 结构体必须使用 __attribute__((packed)) 以确保内存布局与二进制协议一致
     */
    template <typename T>
    inline std::vector<uint8_t> toVector(const T &data)
    {
        // 创建与结构体大小相同的字节向量
        std::vector<uint8_t> packet(sizeof(T));
        // 将结构体内容复制为字节数组
        std::copy(
            reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
            packet.begin());
        return packet;
    }
}// namespace connection_layer

#endif // CONNECTION_LAYER__DATA_PACKET_HPP_
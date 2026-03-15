#include <stdint.h>
#include <string.h>

#define SEND_FRAME_HEADER1 0xAA
#define SEND_FRAME_HEADER2 0x55
#define SEND_MAX_DATA_LEN 64

#define RECEIVE_FRAME_HEADER1 0xAA
#define RECEIVE_FRAME_HEADER2 0x55
#define RECEIVE_MAX_DATA_LEN 8                               // 数据区最大长度（两个float）
#define RECEIVE_FRAME_BUFFER_SIZE (2 + 2 + RECEIVE_MAX_DATA_LEN + 1) // 最大帧长

// 接收状态机变量
static uint8_t rx_state = 0;                 // 当前状态
static uint8_t rx_buffer[RECEIVE_FRAME_BUFFER_SIZE]; // 缓存一帧数据
static uint16_t rx_index = 0;                // 当前写入位置
static uint16_t expected_data_len = 0;       // 从长度字段解析出的数据区长度
static uint8_t rx_checksum = 0;              // 实时累加校验和

// 解析出的数据
static float received_vx, received_vy;
static volatile uint8_t data_ready = 0; // 新数据到达标志

static inline void pack_float(uint8_t *buf, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    buf[0] = (uint8_t)(u & 0xFF);
    buf[1] = (uint8_t)((u >> 8) & 0xFF);
    buf[2] = (uint8_t)((u >> 16) & 0xFF);
    buf[3] = (uint8_t)((u >> 24) & 0xFF);
}

/// @brief 发送数据
/// 参数为各个字段的值，添加新的值在末尾追加，在数据区追加相同处理逻辑
void send_data(float fr, float fl, float rl, float rr,
               float hp, float gain, float progress, float remaining)
{
    uint8_t tx_buffer[SEND_MAX_DATA_LEN];
    uint16_t idx = 0;

    // 帧头
    tx_buffer[idx++] = SEND_FRAME_HEADER1;
    tx_buffer[idx++] = SEND_FRAME_HEADER2;

    // 长度占位
    uint16_t len_field_pos = idx;
    idx += 2;

    // 数据区起始
    uint16_t data_start_idx = idx;

    // 数据区
    uint16_t definition = 0;
    pack_float(tx_buffer + idx, fr);
    idx += 4;
    pack_float(tx_buffer + idx, fl);
    idx += 4;
    pack_float(tx_buffer + idx, rr);
    idx += 4;
    pack_float(tx_buffer + idx, rl);
    idx += 4;
    pack_float(tx_buffer + idx, hp);
    idx += 4;
    pack_float(tx_buffer + idx, gain);
    idx += 4;
    pack_float(tx_buffer + idx, progress);
    idx += 4;
    pack_float(tx_buffer + idx, remaining);
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
    Usb_Sent(tx_buffer, idx);
}

/// @brief 接收数据
void receive_data(uint8_t byte)
{
    switch (rx_state)
    {
    case 0: // 等待帧头1
        if (byte == RECEIVE_FRAME_HEADER1)
        {
            rx_state = 1;
            rx_checksum = byte; // 开始累加
            rx_buffer[0] = byte;
            rx_index = 1;
        }
        break;

    case 1: // 等待帧头2
        if (byte == RECEIVE_FRAME_HEADER2)
        {
            rx_state = 2;
            rx_checksum += byte;
            rx_buffer[rx_index++] = byte;
        }
        else
        {
            rx_state = 0;
            if (byte == RECEIVE_FRAME_HEADER1)
            {
                rx_state = 1;
                rx_checksum = byte;
                rx_buffer[0] = byte;
                rx_index = 1;
            }
        }
        break;

    case 2: // 等待长度低字节
        rx_checksum += byte;
        rx_buffer[rx_index++] = byte;
        expected_data_len = byte; // 暂存低8位
        rx_state = 3;
        break;

    case 3: // 等待长度高字节
        rx_checksum += byte;
        rx_buffer[rx_index++] = byte;
        expected_data_len |= (uint16_t)byte << 8;

        // 检查长度是否合法
        if (expected_data_len > RECEIVE_MAX_DATA_LEN)
        {
            rx_state = 0;
        }
        else if (expected_data_len == 0)
        {
            rx_state = 5;
        }
        else
        {
            rx_state = 4;
        }
        break;

    case 4: // 接收数据区
        rx_checksum += byte;
        rx_buffer[rx_index++] = byte;

        // 检查数据区是否接收完毕
        if (rx_index - (2 + 2) >= expected_data_len)
        {
            rx_state = 5;
        }
        break;

    case 5: // 等待校验和
    {
        uint8_t received_checksum = byte;
        if (received_checksum == rx_checksum)
        {
            uint8_t *data_ptr = &rx_buffer[4];

            uint32_t tmp;
            tmp = (uint32_t)data_ptr[0] |
                  (uint32_t)data_ptr[1] << 8 |
                  (uint32_t)data_ptr[2] << 16 |
                  (uint32_t)data_ptr[3] << 24;
            memcpy(&received_vx, &tmp, 4);

            tmp = (uint32_t)data_ptr[4] |
                  (uint32_t)data_ptr[5] << 8 |
                  (uint32_t)data_ptr[6] << 16 |
                  (uint32_t)data_ptr[7] << 24;
            memcpy(&received_vy, &tmp, 4);

            data_ready = 1; // 标记数据已更新
        }


        rx_state = 0;
        break;
    }

    default:
        rx_state = 0;
        break;
    }
}

/// @brief 获取最新接收到的有效速度数据
void get_received_data(float *vx, float *vy)
{
    if (data_ready)
    {
        *vx = received_vx;
        *vy = received_vy;
        data_ready = 0;
    }
}
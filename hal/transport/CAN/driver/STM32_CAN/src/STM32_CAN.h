#ifndef STM32_CAN_H
#define STM32_CAN_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <stm32f1xx.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------------- CAN 消息格式和类型 -----------------
// 标准 ID / 扩展 ID
typedef enum {
    STANDARD_FORMAT = 0,  // 标准 11 位 ID
    EXTENDED_FORMAT       // 扩展 29 位 ID
} CAN_FORMAT;

// 数据帧 / 远程帧
typedef enum {
    DATA_FRAME = 0,   // 正常数据帧
    REMOTE_FRAME      // 请求远程帧
} CAN_FRAME;

// ----------------- CAN 消息结构体 -----------------
// 用于发送或接收 CAN 消息
typedef struct {
    uint32_t id;        // CAN 消息 ID（11 位或 29 位）
    uint8_t  data[8];   // 数据字段（最多 8 字节）
    uint8_t  len;       // 数据长度
    uint8_t  format;    // 消息格式，使用 CAN_FORMAT
    uint8_t  type;      // 消息类型，使用 CAN_FRAME
} CAN_msg_t;

// ----------------- CAN 波特率时间配置 -----------------
// 用于计算和配置 CAN 总线时序参数
typedef struct {
    uint16_t baud_rate_prescaler;       // 波特率分频器
    uint8_t  time_segment_1;            // 时间段 1（BS1）
    uint8_t  time_segment_2;            // 时间段 2（BS2）
    uint8_t  resynchronization_jump_width; // 同步跳宽度（RJW）
} CAN_bit_timing_config_t;

// ----------------- 公共函数 -----------------

// 初始化 CAN 总线
// bitrate: CAN 总线速度
// remap: GPIO 引脚映射选项（0/2/3 对应不同管脚）
// 返回值: true 初始化成功，false 初始化失败
bool CANInit(uint32_t bitrate, int remap);

// 启动 CAN 总线
bool CANStart(void);
// 停止/注销 CAN
void CANDeinit(void);
// 发送 CAN 消息
// CAN_tx_msg: 待发送消息结构体指针
bool CANSend(CAN_msg_t* CAN_tx_msg);

// 接收 CAN 消息
// CAN_rx_msg: 接收缓冲结构体指针，函数会将接收到的数据填充进去
bool CANReceive(CAN_msg_t* CAN_rx_msg);

// 查询 FIFO0 中是否有未处理消息
// 返回值: 可用消息数量（0/1/2）
uint8_t CANMsgAvail(void);

#ifdef __cplusplus
}
#endif

#endif // STM32_CAN_H
#include "STM32_CAN.h"
#include <string.h> // 用于 memset 清空结构体


// ----------------- 错误码定义 -----------------
#define CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE     1000  // 不支持的波特率错误码

// ----------------- CAN ID 掩码 -----------------
#define CAN_EXT_ID_MASK                 0x1FFFFFFFU  // 扩展ID掩码（29位）
#define CAN_STD_ID_MASK                 0x000007FFU  // 标准ID掩码（11位）

// ----------------- CAN 寄存器位定义 -----------------
// TIR = Tx Mailbox Identifier Register，RIR = Rx FIFO Identifier Register
#define STM32_CAN_TIR_TXRQ              (1U << 0U)  // 发送请求位
#define STM32_CAN_RIR_RTR               (1U << 1U)  // 远程帧位
#define STM32_CAN_RIR_IDE               (1U << 2U)  // 扩展 ID 位
#define STM32_CAN_TIR_RTR               (1U << 1U)  // 发送远程帧位
#define STM32_CAN_TIR_IDE               (1U << 2U)  // 发送扩展 ID 位

// ----------------- 内部函数声明 -----------------
static int16_t ComputeCANTimings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    CAN_bit_timing_config_t* const out_timings
);
// 计算 CAN 波特率分频和时间段，返回 0 表示成功，负值表示不支持该波特率

// ----------------- 初始化 CAN -----------------
bool CANInit(uint32_t bitrate, int remap)
{
    // 1. 开启 CAN 外设时钟和 AFIO 时钟
    RCC->APB1ENR |= 0x2000000UL;  // 开启 CAN1 外设时钟
    RCC->APB2ENR |= 0x1UL;        // 开启 AFIO 时钟
    AFIO->MAPR &= 0xFFFF9FFF;     // 清除 CAN remap 设置

    // 2. 根据 remap 配置对应 GPIO
    if(remap == 0){
        // 默认映射 PA11 = RX, PA12 = TX
        RCC->APB2ENR |= 0x4UL; 
        GPIOA->CRH &= ~(0xFF000UL);
        GPIOA->CRH |= 0xB8FFFUL;
        GPIOA->ODR |= 0x1UL << 11; // 上拉 RX
    } else if(remap == 2){
        // PB8/PB9 映射
        AFIO->MAPR |= 0x00004000;
        RCC->APB2ENR |= 0x8UL;
        GPIOB->CRH &= ~(0xFFUL);
        GPIOB->CRH |= 0xB8UL;
        GPIOB->ODR |= 0x1UL << 8;
    } else if(remap == 3){
        // PD0/PD1 映射
        AFIO->MAPR |= 0x00005000;
        RCC->APB2ENR |= 0x20UL;
        GPIOD->CRL &= ~(0xFFUL);
        GPIOD->CRH |= 0xB8UL;
        GPIOD->ODR |= 0x1UL << 0;
    }

    // 3. 进入初始化模式
    CAN1->MCR |= 0x1UL;           // 设置 INRQ 位进入初始化模式
    while(!(CAN1->MSR & 0x1UL));  // 等待确认进入初始化模式

    // 4. 硬件初始化，开启自动重传
    CAN1->MCR = 0x41UL;           // 0x40 = 自动重传使能

    // 5. 计算并配置波特率
    CAN_bit_timing_config_t timings;
    if(ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), bitrate, &timings) != 0)
        return false;  // 不支持波特率

    CAN1->BTR = (((timings.resynchronization_jump_width - 1U) & 3U) << 24U) |
                (((timings.time_segment_1 - 1U) & 15U) << 16U) |
                (((timings.time_segment_2 - 1U) & 7U) << 20U) |
                ((timings.baud_rate_prescaler - 1U) & 1023U);

    return true;
}

// ----------------- 启动 CAN -----------------
bool CANStart(void)
{
    // 配置滤波器，单 32 位 mask 模式
    CAN1->FMR |= 0x1UL;             // 进入滤波器初始化模式
    CAN1->FMR &= 0xFFFFC0FF;
    CAN1->FMR |= 0x1C << 8;         // 滤波器分组
    CAN1->FA1R &= ~(0x1UL);         // 禁用滤波器 0
    CAN1->FS1R |= 0x1UL;            // 单 32 位
    CAN1->FM1R &= ~(0x1UL);         // mask 模式
    CAN1->FFA1R &= ~(0x1UL);        // FIFO0
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->FA1R |= 0x1UL;            // 启用滤波器 0
    CAN1->FMR &= ~(0x1UL);          // 退出滤波器初始化模式

    // 进入正常工作模式
    CAN1->MCR &= ~(0x1UL);          // 清除 INRQ
    uint16_t timeout = 1000;
    for(uint16_t i = 0; i < timeout; i++){
        if(!(CAN1->MSR & 0x1UL)) break;
        delayMicroseconds(1000);
    }

    return !(CAN1->MSR & 0x1UL);    // 返回是否成功进入正常模式
}

// ----------------- 停止/注销 CAN -----------------
void CANDeinit(void)
{
    CAN1->MCR |= 0x1UL;  // 进入初始化模式
    CAN1->MCR &= ~0x1UL; // 清除初始化模式
    CAN1->FA1R = 0;      // 禁用滤波器
    CAN1->FMR &= ~0x1UL; // 退出滤波器初始化模式
}

// ----------------- CAN 发送 -----------------
bool CANSend(CAN_msg_t* CAN_tx_msg)
{
    if(!CAN_tx_msg) return false;

    uint32_t out = 0;
    // 配置 ID
    if(CAN_tx_msg->format == EXTENDED_FORMAT)
        out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
    else
        out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);

    // 配置远程帧
    if(CAN_tx_msg->type == REMOTE_FRAME)
        out |= STM32_CAN_TIR_RTR;

    // 配置数据长度
    CAN1->sTxMailBox[0].TDTR &= ~0xF;
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;

    // 写入数据寄存器（低4字节 + 高4字节）
    CAN1->sTxMailBox[0].TDLR  = (((uint32_t)CAN_tx_msg->data[3]<<24)|
                                 ((uint32_t)CAN_tx_msg->data[2]<<16)|
                                 ((uint32_t)CAN_tx_msg->data[1]<<8) |
                                 ((uint32_t)CAN_tx_msg->data[0]));
    CAN1->sTxMailBox[0].TDHR  = (((uint32_t)CAN_tx_msg->data[7]<<24)|
                                 ((uint32_t)CAN_tx_msg->data[6]<<16)|
                                 ((uint32_t)CAN_tx_msg->data[5]<<8) |
                                 ((uint32_t)CAN_tx_msg->data[4]));

    // 请求发送
    CAN1->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

    volatile int count = 0;
    while(CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000);

    return !(CAN1->sTxMailBox[0].TIR & 0x1UL); // 返回是否发送成功
}

// ----------------- CAN 接收 -----------------
bool CANReceive(CAN_msg_t* CAN_rx_msg)
{
    if(!CAN_rx_msg) return false;

    if(!(CAN1->RF0R & 0x3UL)) return false; // FIFO0 无可用消息

    uint32_t id = CAN1->sFIFOMailBox[0].RIR;

    // 判断 ID 类型
    if((id & STM32_CAN_RIR_IDE) == 0){
        CAN_rx_msg->format = STANDARD_FORMAT;
        CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
    } else {
        CAN_rx_msg->format = EXTENDED_FORMAT;
        CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
    }

    // 判断帧类型
    CAN_rx_msg->type = (id & STM32_CAN_RIR_RTR) ? REMOTE_FRAME : DATA_FRAME;
    CAN_rx_msg->len  = CAN1->sFIFOMailBox[0].RDTR & 0xFUL;

    // 拷贝数据（低4字节 + 高4字节）
    for(uint8_t i = 0; i < 8; i++){
        if(i < 4)
            CAN_rx_msg->data[i] = (CAN1->sFIFOMailBox[0].RDLR >> (i*8)) & 0xFF;
        else
            CAN_rx_msg->data[i] = (CAN1->sFIFOMailBox[0].RDHR >> ((i-4)*8)) & 0xFF;
    }

    // 释放 FIFO0
    CAN1->RF0R |= 0x20UL;
    return true;
}

// ----------------- 查询可接收消息数量 -----------------
uint8_t CANMsgAvail(void)
{
    return CAN1->RF0R & 0x3UL; // FIFO0 前2位表示可用消息数量
}

// ----------------- 计算 CAN 波特率和时间段 -----------------
static int16_t ComputeCANTimings(
    const uint32_t peripheral_clock_rate,
    const uint32_t target_bitrate,
    CAN_bit_timing_config_t* const out_timings
)
{
    if(target_bitrate < 1000 || !out_timings)
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;

    memset(out_timings, 0, sizeof(*out_timings));

    static const uint8_t MaxBS1 = 16;  // 最大时间段1
    static const uint8_t MaxBS2 = 8;   // 最大时间段2
    const uint8_t max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;
    const uint16_t MaxSamplePointPermill = 900; // 最大采样点比例（‰）

    uint32_t prescaler_bs = peripheral_clock_rate / target_bitrate;
    uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1);

    // 找到可整除的时间段总和
    while((prescaler_bs % (1U + bs1_bs2_sum)) != 0){
        if(bs1_bs2_sum <= 2) return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;
        bs1_bs2_sum--;
    }

    uint32_t prescaler = prescaler_bs / (1U + bs1_bs2_sum);
    if(prescaler < 1 || prescaler > 1024) return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;

    uint8_t bs1 = (uint8_t)(((7*bs1_bs2_sum-1)+4)/8);
    uint8_t bs2 = bs1_bs2_sum - bs1;

    const uint16_t sample_point_permill = (uint16_t)(1000U * (1U + bs1) / (1U + bs1 + bs2));
    if(sample_point_permill > MaxSamplePointPermill){
        bs1 = (uint8_t)((7*bs1_bs2_sum-1)/8);
        bs2 = bs1_bs2_sum - bs1;
    }

    if(bs1 < 1 || bs1 > MaxBS1 || bs2 < 1 || bs2 > MaxBS2) 
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;

    out_timings->baud_rate_prescaler = (uint16_t)prescaler;
    out_timings->resynchronization_jump_width = 1;  // RJW = 1
    out_timings->time_segment_1 = bs1;
    out_timings->time_segment_2 = bs2;

    return 0;
}

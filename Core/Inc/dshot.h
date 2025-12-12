#ifndef __DSHOT_H
#define __DSHOT_H

#include "stm32f1xx_hal.h"

// DShot 相关定义
#define DSHOT_PACKET_LENGTH     16      // DShot 数据包长度 (位)
#define DSHOT_MIN_THROTTLE      48      // DShot 油门最小有效值 (DSHOT_MIN_THROTTLE = 0表示停止，这里为48是最小油门)
#define DSHOT_MAX_THROTTLE      2047    // DShot 油门最大值

// DShot 脉冲宽度 (假设DShot600, 定时器频率为72MHz)
// DShot600: 1.67us/bit, 0: 0.625us, 1: 1.25us
// 定时器计数周期为120 (72MHz / 600kHz = 120)
// 0脉冲宽度: 45 (0.625us * 72MHz = 45)
// 1脉冲宽度: 90 (1.25us * 72MHz = 90)
#define DSHOT_0_PULSE           45
#define DSHOT_1_PULSE           90
#define DSHOT_TIMER_PERIOD      120     // 定时器重装载值

// 函数原型
void DShot_Init(TIM_HandleTypeDef *htim, uint32_t channel_1, uint32_t channel_2, uint32_t channel_3, uint32_t channel_4);
void DShot_SetThrottle(uint8_t motor_idx, uint16_t throttle);
void DShot_SendPacket(uint8_t motor_idx, uint16_t throttle, uint8_t telemetry_request);

#endif /* __DSHOT_H */

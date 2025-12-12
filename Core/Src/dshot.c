#include "dshot.h"

// 假设的定时器句柄和通道数组
static TIM_HandleTypeDef *DShot_htim;
static uint32_t DShot_channels[4];

// 存储DShot数据包的数组，每个电机一个
static uint16_t dshot_packet_data[4][DSHOT_PACKET_LENGTH];

// 计算DShot校验和和遥测位
static uint16_t DShot_CalculateChecksum(uint16_t value, uint8_t telemetry)
{
    uint16_t checksum = 0;
    uint16_t data = (value << 1) | (telemetry & 0x01);
    checksum ^= (data >> 11);
    checksum ^= (data >> 7);
    checksum ^= (data >> 3);
    return (data << 4) | (checksum & 0x0F);
}

// DShot 初始化
void DShot_Init(TIM_HandleTypeDef *htim, uint32_t channel_1, uint32_t channel_2, uint32_t channel_3, uint32_t channel_4)
{
    DShot_htim = htim;
    DShot_channels[0] = channel_1;
    DShot_channels[1] = channel_2;
    DShot_channels[2] = channel_3;
    DShot_channels[3] = channel_4;

    // 配置定时器为PWM模式
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // 初始占空比为0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // 初始化所有DShot通道
    for (int i = 0; i < 4; i++)
    {
        if (HAL_TIM_PWM_ConfigChannel(DShot_htim, &sConfigOC, DShot_channels[i]) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_TIM_PWM_Start(DShot_htim, DShot_channels[i]);
        // 设置定时器周期
        __HAL_TIM_SET_AUTORELOAD(DShot_htim, DSHOT_TIMER_PERIOD);
    }
}

// 发送DShot数据包
void DShot_SendPacket(uint8_t motor_idx, uint16_t throttle, uint8_t telemetry_request)
{
    if (motor_idx >= 4) return; // 检查电机索引

    // 限幅油门值
    if (throttle < DSHOT_MIN_THROTTLE) throttle = 0; // 小于最小油门值则停止
    if (throttle > DSHOT_MAX_THROTTLE) throttle = DSHOT_MAX_THROTTLE;

    // 计算带校验和的数据包
    uint16_t dshot_value = DShot_CalculateChecksum(throttle, telemetry_request);

    // 将16位数据转换为脉冲宽度序列
    for (int i = 0; i < DSHOT_PACKET_LENGTH; i++)
    {
        if ((dshot_value >> (DSHOT_PACKET_LENGTH - 1 - i)) & 0x01)
        {
            dshot_packet_data[motor_idx][i] = DSHOT_1_PULSE;
        }
        else
        {
            dshot_packet_data[motor_idx][i] = DSHOT_0_PULSE;
        }
    }

    // 通过直接修改CCR寄存器来发送DShot信号
    // 这里需要一个精确的定时中断或DMA来触发每个脉冲的更新
    // 简化实现：直接在这里循环发送，这会导致阻塞，实际应用中不能这样做
    // 理想情况下，DShot信号的发送应该在定时器中断或DMA回调中完成

    // 为了演示，这里只是一个简化的占位符，实际需要更高级的定时器/DMA配置
    // 真实的DShot发送需要在一个固定频率的循环中，或者使用DMA来自动更新CCR寄存器

    // 实际实现：
    // 在一个定时器更新中断中，每次中断时发送DShot_PACKET_LENGTH个脉冲
    // 或者使用DMA将dshot_packet_data数组传输到CCR寄存器中

    // 例如，如果使用DMA：
    // HAL_TIM_PWM_Start_DMA(DShot_htim, DShot_channels[motor_idx], (uint32_t*)dshot_packet_data[motor_idx], DSHOT_PACKET_LENGTH);

    // 这里我们只是模拟，并假设每次调用都会更新电机PWM。
    // 实际的DShot驱动需要更精细的定时器和DMA控制。
    for (int i = 0; i < DSHOT_PACKET_LENGTH; i++)
    {
        __HAL_TIM_SET_COMPARE(DShot_htim, DShot_channels[motor_idx], dshot_packet_data[motor_idx][i]);
        // 等待下一个位周期 (这里只是模拟延时，实际应由定时器硬件控制)
        HAL_Delay(1); // 模拟延时，实际DShot不需要这么长的延时
    }
    __HAL_TIM_SET_COMPARE(DShot_htim, DShot_channels[motor_idx], 0); // 发送完后将PWM置低
}

// 设置电机油门
void DShot_SetThrottle(uint8_t motor_idx, uint16_t throttle)
{
    DShot_SendPacket(motor_idx, throttle, 0); // 默认不请求遥测
}

#include "sx1281.h"
#include "main.h" // For HAL_Delay

// 内部函数：等待BUSY引脚变为低电平
static void SX1281_WaitOnBusy(GPIO_TypeDef *busy_port, uint16_t busy_pin)
{
    while(HAL_GPIO_ReadPin(busy_port, busy_pin) == GPIO_PIN_SET);
}

// 内部函数：发送命令
static HAL_StatusTypeDef SX1281_SendCmd(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint8_t cmd, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    SX1281_WaitOnBusy(busy_port, busy_pin);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    if (data != NULL && len > 0)
    {
        status = HAL_SPI_Transmit(hspi, data, len, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return status;
}

// 内部函数：发送命令并接收数据
static HAL_StatusTypeDef SX1281_SendCmdReceive(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint8_t cmd, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len)
{
    HAL_StatusTypeDef status;

    SX1281_WaitOnBusy(busy_port, busy_pin);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    if (tx_data != NULL && tx_len > 0)
    {
        status = HAL_SPI_Transmit(hspi, tx_data, tx_len, HAL_MAX_DELAY);
        if (status != HAL_OK) return status;
    }
    
    if (rx_data != NULL && rx_len > 0)
    {
        uint8_t dummy_byte = 0xFF;
        status = HAL_SPI_TransmitReceive(hspi, &dummy_byte, rx_data, rx_len, HAL_MAX_DELAY);
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return status;
}

// SX1281 初始化
HAL_StatusTypeDef SX1281_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *reset_port, uint16_t reset_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin)
{
    HAL_StatusTypeDef status;
    uint8_t data;

    // 硬件复位
    HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_RESET);
    HAL_Delay(10); // 至少100us，这里给10ms
    HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_SET);
    HAL_Delay(10); // 等待芯片启动

    // 切换到Standby RC模式
    status = SX1281_SendCmd(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_SET_STANDBY, NULL, 0);
    if (status != HAL_OK) return status;

    // 读取芯片ID (示例，实际SX1281可能没有简单的芯片ID寄存器)
    // status = SX1281_ReadRegister(hspi, cs_port, cs_pin, SX1281_REG_CHIP_ID, &data);
    // if (status != HAL_OK) return status;
    // if (data != expected_chip_id) return HAL_ERROR; 

    // 其他初始化配置，例如设置射频频率、LoRa模式参数、数据包参数等
    // 这些配置会很复杂，需要查阅SX1281数据手册和ELRS实现细节

    // 示例：设置LoRa调制参数 (仅为示意，实际值需根据ELRS协议确定)
    uint8_t lora_mod_params[] = {0x07, 0x0C, 0x01}; // SF7, BW125kHz, CR4/5 (示例值)
    status = SX1281_SendCmd(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_SET_LORA_MODULATION_PARAMS, lora_mod_params, sizeof(lora_mod_params));
    if (status != HAL_OK) return status;

    // 示例：设置数据包参数
    uint8_t packet_params[] = {0x00, ELRS_PACKET_SIZE, 0x00, 0x00}; // preamble length=8, payload_len, crc_enable, invert_iq_off
    status = SX1281_SendCmd(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_SET_PACKET_PARAMS, packet_params, sizeof(packet_params));
    if (status != HAL_OK) return status;

    // 进入Rx模式
    status = SX1281_SetRx(hspi, cs_port, cs_pin, 0xFFFFFFFF); // 永不超时
    if (status != HAL_OK) return status;

    return HAL_OK;
}

// 读取SX1281寄存器
HAL_StatusTypeDef SX1281_ReadRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint16_t reg_addr, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_addr[2];
    tx_addr[0] = (uint8_t)(reg_addr >> 8);
    tx_addr[1] = (uint8_t)reg_addr;

    status = SX1281_SendCmdReceive(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_READ_REGISTER, tx_addr, 2, data, 1);
    return status;
}

// 写入SX1281寄存器
HAL_StatusTypeDef SX1281_WriteRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint16_t reg_addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[3];
    tx_buffer[0] = (uint8_t)(reg_addr >> 8);
    tx_buffer[1] = (uint8_t)reg_addr;
    tx_buffer[2] = data;

    status = SX1281_SendCmd(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_WRITE_REGISTER, tx_buffer, 3);
    return status;
}

// 读取SX1281 FIFO缓冲区
HAL_StatusTypeDef SX1281_ReadBuffer(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint8_t offset, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_offset[1];
    tx_offset[0] = offset;
    status = SX1281_SendCmdReceive(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_READ_BUFFER, tx_offset, 1, data, len);
    return status;
}

// 写入SX1281 FIFO缓冲区
HAL_StatusTypeDef SX1281_WriteBuffer(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint8_t offset, uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[256]; // 假设最大缓冲区大小

    if (len + 1 > sizeof(tx_buffer)) return HAL_ERROR; // 检查缓冲区溢出

    tx_buffer[0] = offset;
    memcpy(&tx_buffer[1], data, len);

    status = SX1281_SendCmd(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_WRITE_BUFFER, tx_buffer, len + 1);
    return status;
}

// 设置SX1281为Standby模式
HAL_StatusTypeDef SX1281_SetStandby(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin)
{
    return SX1281_SendCmd(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_SET_STANDBY, NULL, 0);
}

// 设置SX1281为Rx模式
HAL_StatusTypeDef SX1281_SetRx(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint32_t timeout)
{
    uint8_t timeout_bytes[3];
    timeout_bytes[0] = (uint8_t)(timeout >> 16);
    timeout_bytes[1] = (uint8_t)(timeout >> 8);
    timeout_bytes[2] = (uint8_t)timeout;

    return SX1281_SendCmd(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_CMD_SET_RX, timeout_bytes, 3);
}

// ELRS CRC8 计算 (示例实现，需要与ELRS实际使用的多项式和初始值匹配)
static uint8_t ELRS_CalculateCRC8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0; // 初始CRC值，可能需要根据ELRS协议指定
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ ELRS_CRC8_POLY;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ELRS CRC16 计算 (CRC16-IBM, Poly: 0x8005)
static uint16_t ELRS_CalculateCRC16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0; // 初始CRC值，ELRS可能使用非零初始值
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ ELRS_CRC16_POLY;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc; // 最终可能需要异或一个值
}

// 验证OTA数据包的CRC
static bool OtaValidatePacketCrc(OTA_Packet_s *otaPktPtr)
{
    // 假设CRC16位于数据包的末尾，不包含在CRC计算范围内
    uint16_t expected_crc = otaPktPtr->crc16;
    uint16_t calculated_crc = ELRS_CalculateCRC16((uint8_t*)otaPktPtr, ELRS_PACKET_SIZE - sizeof(uint16_t));

    return calculated_crc == expected_crc;
}

// 解包ELRS通道数据
static bool OtaUnpackChannelData(OTA_Packet_s *otaPktPtr, uint16_t *channels_out)
{
    if (otaPktPtr->type != CRSF_FRAME_TYPE_RC_CHANNELS) return false;

    const uint8_t *rc_data = otaPktPtr->rc_channels_packet.rc_data;

    // ELRS通常使用11位通道数据，16个通道
    // 每个通道11位，共16 * 11 = 176位 = 22字节
    // 字节排列顺序: ch0(lsb), ch0(msb)&ch1(lsb), ch1(msb)&ch2(lsb), ...

    channels_out[0] = (rc_data[0] | (rc_data[1] << 8)) & 0x07FF; // Ch0
    channels_out[1] = ((rc_data[1] >> 3) | (rc_data[2] << 5)) & 0x07FF; // Ch1
    channels_out[2] = ((rc_data[2] >> 6) | (rc_data[3] << 2) | (rc_data[4] << 10)) & 0x07FF; // Ch2
    channels_out[3] = ((rc_data[4] >> 1) | (rc_data[5] << 7)) & 0x07FF; // Ch3
    channels_out[4] = ((rc_data[5] >> 4) | (rc_data[6] << 4)) & 0x07FF; // Ch4
    channels_out[5] = ((rc_data[6] >> 7) | (rc_data[7] << 1) | (rc_data[8] << 9)) & 0x07FF; // Ch5
    channels_out[6] = ((rc_data[8] >> 2) | (rc_data[9] << 6)) & 0x07FF; // Ch6
    channels_out[7] = ((rc_data[9] >> 5) | (rc_data[10] << 3)) & 0x07FF; // Ch7
    channels_out[8] = ((rc_data[11]) | (rc_data[12] << 8)) & 0x07FF; // Ch8 (可能从这里重新开始)
    channels_out[9] = ((rc_data[12] >> 3) | (rc_data[13] << 5)) & 0x07FF; // Ch9
    channels_out[10] = ((rc_data[13] >> 6) | (rc_data[14] << 2) | (rc_data[15] << 10)) & 0x07FF; // Ch10
    channels_out[11] = ((rc_data[15] >> 1) | (rc_data[16] << 7)) & 0x07FF; // Ch11
    channels_out[12] = ((rc_data[16] >> 4) | (rc_data[17] << 4)) & 0x07FF; // Ch12
    channels_out[13] = ((rc_data[17] >> 7) | (rc_data[18] << 1) | (rc_data[19] << 9)) & 0x07FF; // Ch13
    channels_out[14] = ((rc_data[19] >> 2) | (rc_data[20] << 6)) & 0x07FF; // Ch14
    channels_out[15] = ((rc_data[20] >> 5) | (rc_data[21] << 3)) & 0x07FF; // Ch15

    return true;
}

// ELRS通道数据获取 (使用OtaUnpackChannelData)
HAL_StatusTypeDef ELRS_Get_Channels(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint16_t *channels)
{
    HAL_StatusTypeDef status;
    OTA_Packet_s rx_packet;

    // TODO: 实际应用中，这里应该通过SX1281的中断来判断是否有新的数据包
    // 当SX1281的IRQ引脚触发时，表示收到数据包
    // 然后在中断服务函数中读取FIFO，或者在主循环中检测中断标志

    // 假设数据包从缓冲区基地址开始，读取整个ELRS_PACKET_SIZE
    status = SX1281_ReadBuffer(hspi, cs_port, cs_pin, busy_port, busy_pin, SX1281_REG_BUFFER_BASE_ADDRESS, (uint8_t*)&rx_packet, ELRS_PACKET_SIZE);
    if (status != HAL_OK) return status;

    // 验证CRC
    if (!OtaValidatePacketCrc(&rx_packet))
    {
        // CRC校验失败
        return HAL_ERROR;
    }

    // 检查数据包类型是否为RC通道数据
    if (rx_packet.type == CRSF_FRAME_TYPE_RC_CHANNELS)
    {
        // 解包通道数据
        if (OtaUnpackChannelData(&rx_packet, channels))
        {
            return HAL_OK;
        }
    }

    return HAL_ERROR; // 未收到有效的RC通道数据包
}

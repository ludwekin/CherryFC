#ifndef __SX1281_H
#define __SX1281_H

#include "stm32f1xx_hal.h"

// SX1281 寄存器地址 (仅列出部分常用寄存器，实际需要更多)
#define SX1281_REG_CHIP_ID              0x02
#define SX1281_REG_PACKET_PAYLOAD_LENGTH 0x07
#define SX1281_REG_RX_GAIN              0x10
#define SX1281_REG_LORA_PREAMBLE_LENGTH 0x11
#define SX1281_REG_LORA_SF_BW           0x12
#define SX1281_REG_BUFFER_BASE_ADDRESS  0x00

// SX1281 命令 (仅列出部分常用命令)
#define SX1281_CMD_GET_STATUS           0xC0
#define SX1281_CMD_SET_STANDBY          0x80
#define SX1281_CMD_WRITE_REGISTER       0x18
#define SX1281_CMD_READ_REGISTER        0x19
#define SX1281_CMD_SET_FS               0xC1
#define SX1281_CMD_SET_RX               0x82
#define SX1281_CMD_WRITE_BUFFER         0x1A
#define SX1281_CMD_READ_BUFFER          0x1B
#define SX1281_CMD_SET_LORA_MODULATION_PARAMS 0x8F
#define SX1281_CMD_SET_PACKET_PARAMS    0x8F // 同一个命令字，需要根据上下文判断

// ELRS 相关
#define ELRS_PACKET_SIZE                64 // 假设ELRS数据包大小，通常CRSF数据包最大为64字节
#define ELRS_CRC8_POLY                  0xD5 // CRSF CRC8多项式 (0xEB)
#define ELRS_CRC16_POLY                 0x8005 // CRSF CRC16多项式

// CRSF 协议相关定义 (简化的，从ELRS源码中提取)
#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8
#define CRSF_FRAME_LENGTH_BYTES         (sizeof(crsf_header_t) + 1)
#define CRSF_FRAME_NOT_COUNTED_BYTES    (sizeof(crsf_header_t) + 1)
#define CRSF_FRAME_TYPE_RC_CHANNELS     0x16
#define CRSF_FRAME_TYPE_LINK_STATISTICS 0x14

typedef struct {
    uint8_t device_addr;
    uint8_t frame_len;
    uint8_t type;
} crsf_header_t;

typedef struct {
    int8_t uplink_RSSI_1;
    int8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI_1;
    uint8_t downlink_RSSI_2;
    uint8_t downlink_Link_quality;
    uint8_t downlink_SNR;
} crsf_link_statistics_t;

// OTA Packet 结构体 (简化版，根据ELRS源码推断)
typedef struct {
    uint8_t type;
    union {
        struct {
            uint8_t packageIndex;
            uint8_t payload[20]; // 简化 payload 长度
            uint8_t stubbornAck;
        } data_ul;
        struct {
            uint8_t nonce;
            uint8_t fhssIndex;
            uint8_t rfRateEnum;
            uint8_t tlmRatio;
            uint8_t UID4;
            uint8_t UID5;
            uint8_t switchEncMode;
            uint8_t geminiMode;
            uint8_t otaProtocol;
            uint8_t newTlmRatio; // 从 sync.newTlmRatio 字段推断
        } sync; // 用于同步数据包
        struct {
            uint8_t rc_data[22]; // RC数据，通常是16通道11位数据
            uint8_t crc8;
        } rc_channels_packet; // RC通道数据包
        // ... 其他数据包类型
    };
    uint16_t crc16; // 假设CRC16
} OTA_Packet_s;

// 函数原型
HAL_StatusTypeDef SX1281_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *reset_port, uint16_t reset_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin);
HAL_StatusTypeDef SX1281_ReadRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint16_t reg_addr, uint8_t *data);
HAL_StatusTypeDef SX1281_WriteRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint16_t reg_addr, uint8_t data);
HAL_StatusTypeDef SX1281_ReadBuffer(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint8_t offset, uint8_t *data, uint8_t len);
HAL_StatusTypeDef SX1281_WriteBuffer(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint8_t offset, uint8_t *data, uint8_t len);
HAL_StatusTypeDef SX1281_SetStandby(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin);
HAL_StatusTypeDef SX1281_SetRx(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint32_t timeout);
HAL_StatusTypeDef ELRS_Get_Channels(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *busy_port, uint16_t busy_pin, uint16_t *channels);
// 新增的ELRS辅助函数原型
static uint8_t ELRS_CalculateCRC8(uint8_t *data, uint8_t len);
static uint16_t ELRS_CalculateCRC16(uint8_t *data, uint16_t len);
static bool OtaValidatePacketCrc(OTA_Packet_s *otaPktPtr);
static bool OtaUnpackChannelData(OTA_Packet_s *otaPktPtr, uint16_t *channels_out);

#endif /* __SX1281_H */

#include "bmi270.h"
#include "main.h" // For HAL_Delay

// 假设的SPI句柄和CS引脚，在main.c中初始化
// extern SPI_HandleTypeDef hspi1;
// extern GPIO_TypeDef *BMI270_CS_GPIO_Port;
// extern uint16_t BMI270_CS_Pin;

// 内部函数，用于SPI读写单个寄存器
static HAL_StatusTypeDef BMI270_SPI_ReadRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg_addr, uint8_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];

    tx_buffer[0] = reg_addr | 0x80; // 最高位设置为1表示读操作

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    *data = rx_buffer[1];
    return status;
}

static HAL_StatusTypeDef BMI270_SPI_WriteRegister(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg_addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[2];

    tx_buffer[0] = reg_addr & ~0x80; // 最高位设置为0表示写操作
    tx_buffer[1] = data;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, tx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return status;
}

// 内部函数，用于SPI读写多个寄存器
static HAL_StatusTypeDef BMI270_SPI_ReadRegisters(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[1];
    uint8_t dummy_byte = 0xFF;

    tx_buffer[0] = reg_addr | 0x80; // 最高位设置为1表示读操作

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, tx_buffer, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    status = HAL_SPI_TransmitReceive(hspi, &dummy_byte, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return status;
}

// BMI270 初始化
HAL_StatusTypeDef BMI270_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;

    // 检查芯片ID
    status = BMI270_ReadID(hspi, cs_port, cs_pin, &chip_id);
    if (status != HAL_OK || chip_id != BMI270_CHIP_ID_VAL) {
        return HAL_ERROR;
    }

    // 软复位 (Optional, but good practice)
    status = BMI270_SPI_WriteRegister(hspi, cs_port, cs_pin, BMI270_CMD_REG, 0xB6); // CMD_SOFT_RESET
    HAL_Delay(50); // 等待复位完成
    if (status != HAL_OK) return status;

    // 等待传感器启动
    HAL_Delay(50); 

    // 进入Normal模式（Power Configuration）
    status = BMI270_SPI_WriteRegister(hspi, cs_port, cs_pin, BMI270_PWR_CONF_REG, 0x00); // 正常模式，所有传感器开启
    if (status != HAL_OK) return status;

    // 加速度计配置
    status = BMI270_SPI_WriteRegister(hspi, cs_port, cs_pin, BMI270_ACC_CONF_REG, 0xA8); // ODR: 1.6kHz, Bandwidth: ODR/2, Normal mode
    if (status != HAL_OK) return status;

    // 陀螺仪配置
    status = BMI270_SPI_WriteRegister(hspi, cs_port, cs_pin, BMI270_GYR_CONF_REG, 0xA8); // ODR: 1.6kHz, Bandwidth: ODR/2, Normal mode
    if (status != HAL_OK) return status;

    // 其他配置... (例如量程，中断等，此处简化)

    return HAL_OK;
}

// 读取BMI270芯片ID
HAL_StatusTypeDef BMI270_ReadID(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t *chip_id)
{
    return BMI270_SPI_ReadRegister(hspi, cs_port, cs_pin, BMI270_CHIP_ID_REG, chip_id);
}

// 读取加速度计和陀螺仪数据
HAL_StatusTypeDef BMI270_ReadAccGyro(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    HAL_StatusTypeDef status;
    uint8_t raw_data[12];

    status = BMI270_SPI_ReadRegisters(hspi, cs_port, cs_pin, BMI270_ACC_X_LSB_REG, raw_data, 12);
    if (status != HAL_OK) return status;

    *acc_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    *acc_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    *acc_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    *gyro_x = (int16_t)((raw_data[7] << 8) | raw_data[6]);
    *gyro_y = (int16_t)((raw_data[9] << 8) | raw_data[8]);
    *gyro_z = (int16_t)((raw_data[11] << 8) | raw_data[10]);

    return HAL_OK;
}

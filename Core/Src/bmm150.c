#include "bmm150.h"
#include "main.h" // For HAL_Delay

// 内部函数，用于I2C读写单个寄存器
static HAL_StatusTypeDef BMM150_I2C_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(hi2c, BMM150_I2C_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
    return status;
}

static HAL_StatusTypeDef BMM150_I2C_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(hi2c, BMM150_I2C_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return status;
}

// 内部函数，用于I2C读写多个寄存器
static HAL_StatusTypeDef BMM150_I2C_ReadRegisters(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(hi2c, BMM150_I2C_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
    return status;
}

// BMM150 初始化
HAL_StatusTypeDef BMM150_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id;

    // 检查芯片ID
    status = BMM150_ReadID(hi2c, &chip_id);
    if (status != HAL_OK || chip_id != BMM150_CHIP_ID_VAL) {
        return HAL_ERROR;
    }

    // 开启电源模式
    status = BMM150_I2C_WriteRegister(hi2c, BMM150_POWER_CONTROL_REG, 0x01); // 正常模式
    if (status != HAL_OK) return status;

    HAL_Delay(50); // 等待启动

    // 配置操作模式为Normal mode, ODR: 10Hz (示例)
    status = BMM150_I2C_WriteRegister(hi2c, BMM150_OP_MODE_REG, 0x00); // 正常模式，ODR：10Hz
    if (status != HAL_OK) return status;

    return HAL_OK;
}

// 读取BMM150芯片ID
HAL_StatusTypeDef BMM150_ReadID(I2C_HandleTypeDef *hi2c, uint8_t *chip_id)
{
    return BMM150_I2C_ReadRegister(hi2c, BMM150_CHIP_ID_REG, chip_id);
}

// 读取磁力计数据
HAL_StatusTypeDef BMM150_ReadMag(I2C_HandleTypeDef *hi2c, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z)
{
    HAL_StatusTypeDef status;
    uint8_t raw_data[8]; // X, Y, Z轴各2字节，加上一个R轴的2字节 (此处不使用R轴)

    status = BMM150_I2C_ReadRegisters(hi2c, BMM150_DATA_X_LSB_REG, raw_data, 8);
    if (status != HAL_OK) return status;

    // 磁力计数据是13位的，需要进行处理。这里暂时读取16位并进行符号扩展
    *mag_x = (int16_t)((raw_data[1] << 8) | raw_data[0]) >> 3; // LSB为0-12位，MSB为13-15位 (此处简化，实际需要根据手册处理)
    *mag_y = (int16_t)((raw_data[3] << 8) | raw_data[2]) >> 3;
    *mag_z = (int16_t)((raw_data[5] << 8) | raw_data[4]) >> 1; // Z轴可能不同

    return HAL_OK;
}

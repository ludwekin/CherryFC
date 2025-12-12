#ifndef __BMI270_H
#define __BMI270_H

#include "stm32f1xx_hal.h"

// BMI270 寄存器地址
#define BMI270_CHIP_ID_REG          0x00
#define BMI270_CHIP_ID_VAL          0x24 // 芯片ID值

#define BMI270_ACC_X_LSB_REG        0x0C
#define BMI270_GYR_X_LSB_REG        0x12

// 命令寄存器
#define BMI270_CMD_REG              0x7E

// 配置寄存器
#define BMI270_INIT_CTRL_REG        0x59
#define BMI270_INIT_DATA_ADDR_0     0x5B
#define BMI270_INIT_DATA_ADDR_1     0x5C

#define BMI270_ACC_CONF_REG         0x40
#define BMI270_GYR_CONF_REG         0x42
#define BMI270_PWR_CONF_REG         0x7C

// 函数原型
HAL_StatusTypeDef BMI270_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
HAL_StatusTypeDef BMI270_ReadID(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t *chip_id);
HAL_StatusTypeDef BMI270_ReadAccGyro(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, int16_t *acc_x, int16_t *acc_y, int16_t *acc_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#endif /* __BMI270_H */

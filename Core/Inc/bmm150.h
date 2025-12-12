#ifndef __BMM150_H
#define __BMM150_H

#include "stm32f1xx_hal.h"

// BMM150 寄存器地址
#define BMM150_CHIP_ID_REG          0x40
#define BMM150_CHIP_ID_VAL          0x32 // 芯片ID值

#define BMM150_POWER_CONTROL_REG    0x4B
#define BMM150_OP_MODE_REG          0x4C
#define BMM150_DATA_X_LSB_REG       0x42

// I2C地址 (通常为0x10或0x13)
#define BMM150_I2C_ADDRESS          (0x10 << 1) // 7位地址左移1位

// 函数原型
HAL_StatusTypeDef BMM150_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMM150_ReadID(I2C_HandleTypeDef *hi2c, uint8_t *chip_id);
HAL_StatusTypeDef BMM150_ReadMag(I2C_HandleTypeDef *hi2c, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);

#endif /* __BMM150_H */

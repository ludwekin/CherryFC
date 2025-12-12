// flight_controller_core.h
#ifndef FLIGHT_CONTROLLER_CORE_H
#define FLIGHT_CONTROLLER_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

// ============================================================================
//                       1. 宏定义和常量 (Macros and Constants)
// ============================================================================

// --- 飞行器配置 ---
#define NUM_MOTORS 4 // 四旋翼
#define NUM_RC_CHANNELS 4 // 副翼(Roll), 升降(Pitch), 油门(Throttle), 方向(Yaw)

// --- IMU 配置 ---
#define GYRO_SAMPLE_RATE_HZ 1000 // 陀螺仪采样率 1000 Hz
#define ACC_SAMPLE_RATE_HZ 1000  // 加速度计采样率 1000 Hz
#define IMU_UPDATE_RATE_HZ 250 // IMU 姿态更新率 250 Hz
#define IMU_DT (1.0f / IMU_UPDATE_RATE_HZ) // IMU 更新时间步长

// MPU6050/6000 默认量程 +/- 2000 deg/s for gyro, +/- 8G for accel
// Gyro scale for converting raw ADC to deg/s
#define GYRO_RAW_TO_DPS_SCALE (2000.0f / 32768.0f) // (Full_scale_dps / 2^15)
// Accel scale for converting raw ADC to G
#define ACC_RAW_TO_G_SCALE (8.0f / 32768.0f) // (Full_scale_G / 2^15)

// --- RC 通道配置 (以微秒为单位的 PWM 脉宽) ---
#define RC_MIN_VALUE 1000
#define RC_MID_VALUE 1500
#define RC_MAX_VALUE 2000
#define RC_CHANNEL_RANGE (RC_MAX_VALUE - RC_MIN_VALUE)

// --- PID 配置 ---
#define PID_LOOP_RATE_HZ 250 // PID 控制器循环率 250 Hz
#define PID_DT (1.0f / PID_LOOP_RATE_HZ) // PID 更新时间步长

// 简化 PID 参数 (P, I, D) - 示例值，需要调整
#define PID_ROLL_P 3.0f
#define PID_ROLL_I 0.01f
#define PID_ROLL_D 0.5f

#define PID_PITCH_P 3.0f
#define PID_PITCH_I 0.01f
#define PID_PITCH_D 0.5f

#define PID_YAW_P 5.0f
#define PID_YAW_I 0.0f // 偏航通常不需要积分，或者很小
#define PID_YAW_D 0.0f

#define PID_ANGLE_GAIN 5.0f // 角度模式下，角度误差到期望角速度的转换增益 (度/秒每度误差)
#define PID_MAX_ANGLE_DEG 50.0f // 角度模式下最大倾斜角度 (度)

// --- 电机输出配置 (以微秒为单位的 PWM 脉宽) ---
#define MOTOR_MIN_PWM 1000
#define MOTOR_MAX_PWM 2000
#define MOTOR_IDLE_PWM 1050 // 解锁时的怠速 PWM

// --- 姿态估算 (互补滤波) ---
#define COMPLEMENTARY_FILTER_ALPHA 0.98f // 陀螺仪信任度，加速度计信任度为 1 - ALPHA

// --- 数学常量 ---
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

/* USER CODE BEGIN Private defines */

#define BMI270_CS_GPIO_Port GPIOB
#define BMI270_CS_Pin GPIO_PIN_12

// BMI270 Registers
#define BMI270_CHIP_ID_REG      0x00
#define BMI270_ACC_CONF_REG     0x40
#define BMI270_GYR_CONF_REG     0x42
#define BMI270_CMD_REG          0x7E

// BMI270 Commands
#define BMI270_SOFT_RESET_CMD   0xB6
#define BMI270_ACC_ENABLE_CMD   0x04
#define BMI270_GYR_ENABLE_CMD   0x08

// Function Prototypes for BMI270 SPI communication
void BMI270_SPI_Init(SPI_HandleTypeDef* hspi);
uint8_t BMI270_ReadRegister(uint8_t reg_addr);
void BMI270_WriteRegister(uint8_t reg_addr, uint8_t value);
void BMI270_ChipSelect(uint8_t enable);

// BMM150 I2C Address
#define BMM150_I2C_ADDRESS      0x10 << 1 // 7-bit address 0x10, shifted for 8-bit R/W

// BMM150 Registers
#define BMM150_CHIP_ID_REG      0x40
#define BMM150_POWER_CTRL_REG   0x4B
#define BMM150_OP_MODE_REG      0x4C
#define BMM150_DATA_X_LSB_REG   0x42

// BMM150 Commands/Values
#define BMM150_POWER_CTRL_ENABLE 0x01
#define BMM150_OP_MODE_NORMAL    0x00

// Function Prototypes for BMM150 I2C communication
void BMM150_I2C_Init(I2C_HandleTypeDef* hi2c);
uint8_t BMM150_ReadRegister(uint8_t reg_addr);
void BMM150_WriteRegister(uint8_t reg_addr, uint8_t value);

// BMI270 Sensor Data Structure
typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
} BMI270_SensorData_t;

// Function Prototypes for BMI270 data acquisition
uint8_t BMI270_Init(void);
void BMI270_ReadSensorData(BMI270_SensorData_t* data);

// BMM150 Sensor Data Structure
typedef struct {
    int16_t mag_x, mag_y, mag_z;
} BMM150_SensorData_t;

// Function Prototypes for BMM150 data acquisition
uint8_t BMM150_Init(void);
void BMM150_ReadSensorData(BMM150_SensorData_t* data);

// SX1281 SPI Pins
#define SX1281_CS_GPIO_Port GPIOB
#define SX1281_CS_Pin GPIO_PIN_0 // Example pin, adjust as needed
#define SX1281_RESET_GPIO_Port GPIOA
#define SX1281_RESET_Pin GPIO_PIN_8 // Example pin, adjust as needed
#define SX1281_BUSY_GPIO_Port GPIOA
#define SX1281_BUSY_Pin GPIO_PIN_9 // Example pin, adjust as needed

// SX1281 Registers/Commands (Simplified for placeholder)
#define SX1281_WRITE_REGISTER_CMD 0x18
#define SX1281_READ_REGISTER_CMD  0x18
#define SX1281_SET_STANDBY_CMD    0x80
#define SX1281_PACKET_LENGTH_REG  0x900

// Function Prototypes for SX1281 SPI communication
void SX1281_SPI_Init(SPI_HandleTypeDef* hspi);
void SX1281_ChipSelect(uint8_t enable);
void SX1281_Reset(void);
uint8_t SX1281_ReadRegister(uint16_t reg_addr);
void SX1281_WriteRegister(uint16_t reg_addr, uint8_t value);
void SX1281_WriteCommand(uint8_t cmd);
void SX1281_ReadBuffer(uint8_t offset, uint8_t* buffer, uint8_t size);

// ELRS RC Channels Data Structure (Simplified)
typedef struct {
    uint16_t roll;
    uint16_t pitch;
    uint16_t yaw;
    uint16_t throttle;
    uint16_t aux1;
    uint16_t aux2;
    // ... add more channels as needed
} ELRS_RC_Channels_t;

// Function Prototypes for ELRS reception
uint8_t ELRS_Init(void);
uint8_t ELRS_ReceivePacket(ELRS_RC_Channels_t* channels);

// Dshot Defines (Example: DSHOT300)
#define DSHOT_SPEED             300 // DSHOT300
#define DSHOT_FRAME_LENGTH      16  // 11 data bits + 1 telemetry bit + 4 CRC bits
#define DSHOT_BIT_0_HIGH_TIME   (1 * (SystemCoreClock / (2 * DSHOT_SPEED * 1000000))) // Example for DSHOT300
#define DSHOT_BIT_1_HIGH_TIME   (2 * (SystemCoreClock / (2 * DSHOT_SPEED * 1000000))) // Example for DSHOT300
#define DSHOT_PERIOD            (3 * (SystemCoreClock / (2 * DSHOT_SPEED * 1000000))) // Example for DSHOT300

// Dshot Frame Buffer (for DMA or interrupt driven Dshot)
#define DSHOT_DMA_BUFFER_SIZE   DSHOT_FRAME_LENGTH // 16 bits per frame

// Function Prototypes for Dshot
void Dshot_Timer_Init(TIM_HandleTypeDef* htim_ch1, TIM_HandleTypeDef* htim_ch2, TIM_HandleTypeDef* htim_ch3, TIM_HandleTypeDef* htim_ch4);
void Dshot_SetMotorSpeed(uint8_t motor_id, uint16_t throttle);
void Dshot_GenerateFrame(uint16_t throttle_value, uint32_t* dshot_frame_buffer);

// Attitude Data Structure
typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

// Function Prototypes for Sensor Fusion
void ComplementaryFilter_Init(float dt, float gyro_weight);
void ComplementaryFilter_Update(BMI270_SensorData_t* acc_gyr_data, BMM150_SensorData_t* mag_data, Attitude_t* attitude);

// PID Controller Data Structure
typedef struct {
    float kp, ki, kd;
    float previous_error;
    float integral;
    float output_limit;
} PID_Controller_t;

typedef enum {
    MODE_ACRO, // 角度模式
    MODE_ANGLE // 自稳模式
} FlightMode_t;

/* USER CODE END Private defines */

// ============================================================================
//                       2. 数据结构 (Data Structures)
// ============================================================================

// 3D 向量
typedef struct {
    float x, y, z;
} Vector3f;

// 四元数
typedef struct {
    float w, x, y, z;
} Quaternion;

// 欧拉角 (度)
typedef struct {
    float roll, pitch, yaw;
} EulerAngles;

// IMU 传感器数据
typedef struct {
    int16_t gyro_raw[3]; // 原始陀螺仪数据 (ADC 值)
    float gyro_dps[3];   // 陀螺仪数据 (度/秒)
    int16_t accel_raw[3]; // 原始加速度计数据 (ADC 值)
    float accel_g[3];    // 加速度计数据 (G)
} ImuData;

// PID 控制器结构
typedef struct {
    float kp, ki, kd;
    float prev_error[3]; // 上一次误差
    float integral[3];   // 积分项
} PidController;

// 混控器电机输出
typedef struct {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} MotorMixerCoefficients;

/* Exported types ------------------------------------------------------------*/
typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
} SensorRawData_t;

typedef struct {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
} SensorCalibratedData_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
    float output_limit;
    float integral_limit;
} PID_Controller_t;

typedef enum {
    MODE_ACRO, // 角度模式
    MODE_ANGLE // 自稳模式
} FlightMode_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void FlightController_Init(void);
void FlightController_Loop(void);
HAL_StatusTypeDef FC_ReadSensors(SensorRawData_t *rawData);
void FC_CalibrateSensors(SensorRawData_t *rawData, SensorCalibratedData_t *calibratedData);
void FC_EstimateAttitude(SensorCalibratedData_t *calibratedData, Attitude_t *attitude);
float PID_Calculate(PID_Controller_t *pid, float setpoint, float measured, float dt);
void FC_ControlLoop(Attitude_t *currentAttitude, uint16_t *rc_channels, FlightMode_t flightMode, float *motor_outputs);
HAL_StatusTypeDef FC_ReadBatteryVoltage(float *voltage);

/* USER CODE BEGIN EFP */

// ============================================================================
//                       3. 外部硬件驱动接口 (External Hardware Driver Interfaces)
//    TODO: 在你的 CubeIDE 项目中，实现这些函数。
// ============================================================================

// 获取微秒计时器
extern uint32_t get_micros();

// 获取毫秒计时器
extern uint32_t get_millis();

// 读取 IMU 原始数据
extern void read_imu_raw_data(int16_t* gyro_raw_out, int16_t* accel_raw_out);

// 读取遥控器通道值 (1000-2000 us)
extern void read_rc_receiver_channels(float* channels_out);

// 写入电机 PWM 值 (1000-2000 us)
extern void write_motor_pwms(const float* motor_values);

// ============================================================================
//                       4. 全局变量 (Global Variables)
// ============================================================================

extern ImuData imu_data;
extern float rc_channels[NUM_RC_CHANNELS]; // 遥控器通道值 (1000-2000 us)
extern Quaternion estimated_attitude_quat;
extern EulerAngles estimated_attitude_euler;
extern float motor_output[NUM_MOTORS];     // 电机输出 (1000-2000 us)

// ============================================================================
//                       5. 飞控核心函数声明 (Flight Controller Core Function Declarations)
// ============================================================================

void flight_controller_init();
void flight_controller_main_loop();

#endif // FLIGHT_CONTROLLER_CORE_H

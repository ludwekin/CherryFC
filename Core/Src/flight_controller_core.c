// flight_controller_core.c
#include "flight_controller_core.h" // 包含头文件

// ============================================================================
//                       1. 全局状态变量定义 (Global State Variable Definitions)
// ============================================================================

ImuData imu_data;
float rc_channels[NUM_RC_CHANNELS]; // 遥控器通道值 (1000-2000 us)
Quaternion estimated_attitude_quat;
EulerAngles estimated_attitude_euler;
static PidController pid_controller; // PID 控制器是内部实现细节
float motor_output[NUM_MOTORS];     // 电机输出 (1000-2000 us)

// Quad X 混控矩阵 (油门, 横滚, 俯仰, 偏航)
// 这里的系数是相对于期望的 Roll/Pitch/Yaw 变化量
static const MotorMixerCoefficients quad_x_mixer[NUM_MOTORS] = {
    {1.0f, -1.0f,  1.0f, -1.0f}, // Motor 1 (后右)
    {1.0f, -1.0f, -1.0f,  1.0f}, // Motor 2 (前右)
    {1.0f,  1.0f,  1.0f,  1.0f}, // Motor 3 (后左)
    {1.0f,  1.0f, -1.0f, -1.0f}, // Motor 4 (前左)
};


// ============================================================================
//                       2. 姿态估算 (Attitude Estimation)
// ============================================================================

// 四元数乘法
static Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion qr;
    qr.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    qr.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    qr.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    qr.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return qr;
}

// 四元数归一化
static Quaternion quaternion_normalize(Quaternion q_in) {
    float norm = sqrtf(q_in.w * q_in.w + q_in.x * q_in.x + q_in.y * q_in.y + q_in.z * q_in.z);
    if (norm == 0.0f) {
        return (Quaternion){1.0f, 0.0f, 0.0f, 0.0f}; // 返回单位四元数
    }
    return (Quaternion){q_in.w / norm, q_in.x / norm, q_in.y / norm, q_in.z / norm};
}

// 将四元数转换为欧拉角 (XYZ 顺序，单位：度)
static EulerAngles quaternion_to_euler(Quaternion q_in) {
    EulerAngles angles;
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q_in.w * q_in.x + q_in.y * q_in.z);
    float cosr_cosp = 1.0f - 2.0f * (q_in.x * q_in.x + q_in.y * q_in.y);
    angles.roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q_in.w * q_in.y - q_in.z * q_in.x);
    if (fabsf(sinp) >= 1)
        angles.pitch = copysignf(M_PI / 2.0f, sinp) * RAD_TO_DEG; // Use 90 degrees if out of range
    else
        angles.pitch = asinf(sinp) * RAD_TO_DEG;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q_in.w * q_in.z + q_in.x * q_in.y);
    float cosy_cosp = 1.0f - 2.0f * (q_in.y * q_in.y + q_in.z * q_in.z);
    angles.yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    return angles;
}

// 更新姿态 (简化的互补滤波)
static void update_attitude(float dt) {
    // 陀螺仪数据 (单位: 弧度/秒)
    float gx = imu_data.gyro_dps[0] * DEG_TO_RAD;
    float gy = imu_data.gyro_dps[1] * DEG_TO_RAD;
    float gz = imu_data.gyro_dps[2] * DEG_TO_RAD;

    // 加速度计数据 (单位: G)
    float ax = imu_data.accel_g[0];
    float ay = imu_data.accel_g[1];
    float az = imu_data.accel_g[2];

    // 从加速度计计算 Roll 和 Pitch (假设无偏航)
    // Accel vector representing gravity should ideally be (0, 0, -1) in body frame
    float accel_roll = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    // 互补滤波融合
    // Roll
    estimated_attitude_euler.roll = COMPLEMENTARY_FILTER_ALPHA * (estimated_attitude_euler.roll + gx * dt * RAD_TO_DEG) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;
    // Pitch
    estimated_attitude_euler.pitch = COMPLEMENTARY_FILTER_ALPHA * (estimated_attitude_euler.pitch + gy * dt * RAD_TO_DEG) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;
    // Yaw (仅通过陀螺仪积分)
    estimated_attitude_euler.yaw += gz * dt * RAD_TO_DEG;

    // 欧拉角归一化 (可选，取决于你的使用场景)
    if (estimated_attitude_euler.yaw > 180.0f) estimated_attitude_euler.yaw -= 360.0f;
    if (estimated_attitude_euler.yaw < -180.0f) estimated_attitude_euler.yaw += 360.0f;
}

// ============================================================================
//                       3. PID 控制 (PID Control)
// ============================================================================

// 初始化 PID 控制器
static void pid_init_internal(PidController* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error[0] = pid->prev_error[1] = pid->prev_error[2] = 0.0f;
    pid->integral[0] = pid->integral[1] = pid->integral[2] = 0.0f;
}

// 计算 PID 输出
// axis: 0=Roll, 1=Pitch, 2=Yaw
// setpoint: 期望角速度 (度/秒)
// current_rate: 当前角速度 (度/秒)
static float pid_calculate(PidController* pid, int axis, float setpoint, float current_rate, float dt) {
    float error = setpoint - current_rate;

    // P 项
    float p_term = pid->kp * error;

    // I 项
    pid->integral[axis] += error * dt;
    // 限制积分项，防止积分饱和
    if (pid->integral[axis] > 500.0f) pid->integral[axis] = 500.0f;
    if (pid->integral[axis] < -500.0f) pid->integral[axis] = -500.0f;
    float i_term = pid->ki * pid->integral[axis];

    // D 项
    float derivative = (error - pid->prev_error[axis]) / dt;
    float d_term = pid->kd * derivative;

    // 更新上一次误差
    pid->prev_error[axis] = error;

    return p_term + i_term + d_term;
}

// ============================================================================
//                       4. 飞控核心主循环函数 (Flight Controller Core Loop Functions)
// ============================================================================

// 传感器数据读取和预处理
static void sensors_read_and_process_internal() {
    read_imu_raw_data(imu_data.gyro_raw, imu_data.accel_raw);

    // 将原始陀螺仪数据转换为度/秒
    for (int i = 0; i < 3; i++) {
        imu_data.gyro_dps[i] = (float)imu_data.gyro_raw[i] * GYRO_RAW_TO_DPS_SCALE;
    }
    // 将原始加速度计数据转换为 G
    for (int i = 0; i < 3; i++) {
        imu_data.accel_g[i] = (float)imu_data.accel_raw[i] * ACC_RAW_TO_G_SCALE;
    }

    // TODO: 在这里添加陀螺仪/加速度计的校准、滤波、坐标轴对齐
}

// 遥控器输入处理
static void rc_process_input_internal() {
    read_rc_receiver_channels(rc_channels);

    // 将 RC 原始值 (1000-2000) 映射到 -1.0 到 1.0 的范围 (油门 0.0 到 1.0)
    // Roll, Pitch, Yaw
    for (int i = 0; i < 2; i++) { // Roll, Pitch
        rc_channels[i] = (rc_channels[i] - RC_MID_VALUE) / (RC_CHANNEL_RANGE / 2.0f); // -1.0 to 1.0
    }
    rc_channels[2] = (rc_channels[2] - RC_MIN_VALUE) / RC_CHANNEL_RANGE; // Throttle 0.0 to 1.0
    rc_channels[3] = (rc_channels[3] - RC_MID_VALUE) / (RC_CHANNEL_RANGE / 2.0f); // Yaw -1.0 to 1.0

    // 钳位以防超范围
    for (int i = 0; i < NUM_RC_CHANNELS; i++) {
        if (i == 2) { // 油门
            if (rc_channels[i] < 0.0f) rc_channels[i] = 0.0f;
            if (rc_channels[i] > 1.0f) rc_channels[i] = 1.0f;
        } else { // Roll, Pitch, Yaw
            if (rc_channels[i] < -1.0f) rc_channels[i] = -1.0f;
            if (rc_channels[i] > 1.0f) rc_channels[i] = 1.0f;
        }
    }
}

// 计算 PID 修正量
static void calculate_pid_corrections_internal() {
    // 期望角速度 (Setpoint)
    float desired_rate[3]; // Roll_dps, Pitch_dps, Yaw_dps

    // 判断是否在解锁且摇杆回中状态，此时积分和 D 项需要重置
    bool is_disarmed_and_idle = (rc_channels[2] < 0.1f && fabsf(rc_channels[0]) < 0.1f && fabsf(rc_channels[1]) < 0.1f && fabsf(rc_channels[3]) < 0.1f);

    // --- Roll 和 Pitch (角度模式自稳定) ---
    // 期望姿态 (Roll/Pitch): 由遥控器输入决定期望角度，然后通过 PID_ANGLE_GAIN 转换为期望角速度
    // 如果摇杆回中，期望角度为 0
    float desired_angle_roll = rc_channels[0] * PID_MAX_ANGLE_DEG;
    float desired_angle_pitch = rc_channels[1] * PID_MAX_ANGLE_DEG;

    // 角度误差
    float error_angle_roll = desired_angle_roll - estimated_attitude_euler.roll;
    float error_angle_pitch = desired_angle_pitch - estimated_attitude_euler.pitch;

    // 将角度误差转换为期望的角速度 (度/秒)
    desired_rate[0] = error_angle_roll * PID_ANGLE_GAIN;
    desired_rate[1] = error_angle_pitch * PID_ANGLE_GAIN;

    // --- Yaw (速率模式控制) ---
    // 期望角速度直接来自遥控器输入 (假设 +/- 200 deg/s 的最大偏航速率)
    desired_rate[2] = rc_channels[3] * 200.0f;

    // 计算每个轴的 PID 输出
    if (is_disarmed_and_idle) { // 解锁或怠速时重置积分和 D 项
        pid_controller.integral[0] = pid_controller.integral[1] = pid_controller.integral[2] = 0.0f;
        pid_controller.prev_error[0] = pid_controller.prev_error[1] = pid_controller.prev_error[2] = 0.0f;
    }

    pid_init_internal(&pid_controller, PID_ROLL_P, PID_ROLL_I, PID_ROLL_D);
    float roll_correction = pid_calculate(&pid_controller, 0, desired_rate[0], imu_data.gyro_dps[0], PID_DT);

    pid_init_internal(&pid_controller, PID_PITCH_P, PID_PITCH_I, PID_PITCH_D);
    float pitch_correction = pid_calculate(&pid_controller, 1, desired_rate[1], imu_data.gyro_dps[1], PID_DT);

    pid_init_internal(&pid_controller, PID_YAW_P, PID_YAW_I, PID_YAW_D);
    float yaw_correction = pid_calculate(&pid_controller, 2, desired_rate[2], imu_data.gyro_dps[2], PID_DT);

    // 将 PID 修正量存储，供混控器使用
    // 注意：这里的 motor_output[0-2] 暂时存储的是 PID 修正量，不是最终的电机 PWM
    motor_output[0] = roll_correction;
    motor_output[1] = pitch_correction;
    motor_output[2] = yaw_correction;
}

// 电机混控与输出
static void mix_and_output_motors_internal() {
    float throttle_input = rc_channels[2]; // 油门输入 (0.0 - 1.0)

    // 如果油门很低，且摇杆回中，进入怠速模式 (模拟解锁状态)
    bool is_disarmed_idle = (throttle_input < 0.1f && fabsf(rc_channels[0]) < 0.1f && fabsf(rc_channels[1]) < 0.1f && fabsf(rc_channels[3]) < 0.1f);

    for (int i = 0; i < NUM_MOTORS; i++) {
        float motor_speed_normalized = 0.0f;

        // 计算每个电机的混控输出
        // 这里的 motor_output[0-2] 实际上是之前计算的 roll_correction, pitch_correction, yaw_correction
        motor_speed_normalized =
            throttle_input * quad_x_mixer[i].throttle +
            motor_output[0] * quad_x_mixer[i].roll +   // Roll correction
            motor_output[1] * quad_x_mixer[i].pitch +  // Pitch correction
            motor_output[2] * quad_x_mixer[i].yaw;     // Yaw correction

        // 将归一化后的电机速度钳位到 0.0-1.0 范围
        motor_speed_normalized = fmaxf(0.0f, fminf(1.0f, motor_speed_normalized));

        // 如果处于解锁怠速状态，则输出怠速 PWM，否则按比例输出
        if (is_disarmed_idle) {
             motor_output[i] = MOTOR_IDLE_PWM; // 未解锁，保持怠速
        } else {
             motor_output[i] = MOTOR_MIN_PWM + motor_speed_normalized * (MOTOR_MAX_PWM - MOTOR_MIN_PWM);
        }

        // 最终钳位到 PWM 输出范围
        motor_output[i] = fmaxf(MOTOR_MIN_PWM, fminf(MOTOR_MAX_PWM, motor_output[i]));
    }

    write_motor_pwms(motor_output);
}


// 主飞控初始化函数
void flight_controller_init() {
    // 初始化姿态估计
    estimated_attitude_quat = (Quaternion){1.0f, 0.0f, 0.0f, 0.0f}; // 初始姿态为单位四元数
    estimated_attitude_euler = (EulerAngles){0.0f, 0.0f, 0.0f};    // 初始欧拉角为 0

    // 初始化 PID 控制器 (只初始化结构体，实际参数在 loop 中设置)
    pid_init_internal(&pid_controller, 0, 0, 0);

    // TODO: 其他外设初始化 (UART, SPI, 定时器等)
    // 这部分通常由 CubeIDE 生成，并在此处调用相应的 HAL/LL 初始化函数
}

// 主飞控循环 (在你的主函数中周期性调用)
void flight_controller_main_loop() {
    static uint32_t last_imu_update_micros = 0;
    static uint32_t last_pid_update_micros = 0;

    uint32_t current_micros = get_micros();

    // IMU 更新 (姿态估计)
    if ((current_micros - last_imu_update_micros) >= (1000000 / IMU_UPDATE_RATE_HZ)) {
        sensors_read_and_process_internal(); // 读取并预处理传感器数据
        update_attitude(IMU_DT);             // 更新姿态估计
        last_imu_update_micros = current_micros;
    }

    // PID 控制和电机输出
    if ((current_micros - last_pid_update_micros) >= (1000000 / PID_LOOP_RATE_HZ)) {
        rc_process_input_internal();         // 处理遥控器输入
        calculate_pid_corrections_internal(); // 计算 PID 修正量
        mix_and_output_motors_internal();     // 混控电机并输出 PWM
        last_pid_update_micros = current_micros;
    }
}

/**
  * @brief  Initializes the Complementary Filter.
  * @param  dt: Time step in seconds.
  * @param  gyro_weight: Weight for gyroscope data (0.0 to 1.0).
  * @retval None
  */
void ComplementaryFilter_Init(float dt, float gyro_weight)
{
  complementary_filter_dt = dt;
  complementary_filter_gyro_weight = gyro_weight;
  current_attitude.roll = 0.0f;
  current_attitude.pitch = 0.0f;
  current_attitude.yaw = 0.0f;
}

/**
  * @brief  Updates the attitude using a Complementary Filter.
  * @param  acc_gyr_data: Pointer to BMI270_SensorData_t containing accelerometer and gyroscope data.
  * @param  mag_data: Pointer to BMM150_SensorData_t containing magnetometer data.
  * @param  attitude: Pointer to Attitude_t structure to store the estimated attitude.
  * @retval None
  */
void ComplementaryFilter_Update(BMI270_SensorData_t* acc_gyr_data, BMM150_SensorData_t* mag_data, Attitude_t* attitude)
{
  // Convert accelerometer data to roll and pitch angles
  // Assuming accelerometer measures gravity along Z-axis when level
  float acc_roll = atan2f(acc_gyr_data->acc_y, acc_gyr_data->acc_z) * (180.0f / M_PI);
  float acc_pitch = atan2f(-acc_gyr_data->acc_x, sqrtf(acc_gyr_data->acc_y * acc_gyr_data->acc_y + acc_gyr_data->acc_z * acc_gyr_data->acc_z)) * (180.0f / M_PI);

  // Convert gyroscope data to angular velocities (degrees/sec)
  // Scale factors depend on the BMI270 configuration
  float gyr_roll_rate = acc_gyr_data->gyr_x * 0.001f; // Example scale factor
  float gyr_pitch_rate = acc_gyr_data->gyr_y * 0.001f;
  float gyr_yaw_rate = acc_gyr_data->gyr_z * 0.001f;

  // Integrate gyroscope data to get angular position
  attitude->roll += gyr_roll_rate * complementary_filter_dt;
  attitude->pitch += gyr_pitch_rate * complementary_filter_dt;
  attitude->yaw += gyr_yaw_rate * complementary_filter_dt;

  // Apply complementary filter
  attitude->roll = complementary_filter_gyro_weight * attitude->roll + (1.0f - complementary_filter_gyro_weight) * acc_roll;
  attitude->pitch = complementary_filter_gyro_weight * attitude->pitch + (1.0f - complementary_filter_gyro_weight) * acc_pitch;

  // Magnetometer for yaw correction (simplified)
  // This part is highly dependent on calibration and surrounding magnetic fields.
  // For a full implementation, consider tilt compensation and more advanced algorithms.
  float mag_yaw = atan2f(-mag_data->mag_y, mag_data->mag_x) * (180.0f / M_PI);
  // A simple blending (needs more robust approach)
  attitude->yaw = complementary_filter_gyro_weight * attitude->yaw + (1.0f - complementary_filter_gyro_weight) * mag_yaw;

  // Store the updated attitude
  current_attitude = *attitude;
}

/**
  * @brief  Initializes a PID controller.
  * @param  pid: Pointer to the PID_Controller_t structure.
  * @param  kp, ki, kd: PID gains.
  * @param  output_limit: Maximum absolute output value.
  * @retval None
  */
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, float output_limit)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->previous_error = 0.0f;
  pid->integral = 0.0f;
  pid->output_limit = output_limit;
}

/**
  * @brief  Updates the PID controller and calculates the output.
  * @param  pid: Pointer to the PID_Controller_t structure.
  * @param  setpoint: Desired value.
  * @param  measured_value: Actual measured value.
  * @param  dt: Time step in seconds.
  * @retval PID output.
  */
float PID_Update(PID_Controller_t* pid, float setpoint, float measured_value, float dt)
{
  float error = setpoint - measured_value;
  pid->integral += error * dt;
  // Limit integral wind-up
  if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
  if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;

  float derivative = (error - pid->previous_error) / dt;
  float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  pid->previous_error = error;

  // Limit output
  if (output > pid->output_limit) output = pid->output_limit;
  if (output < -pid->output_limit) output = -pid->output_limit;

  return output;
}

/**
  * @brief  Initializes the flight controller.
  * @param  dt: Time step in seconds for control loop.
  * @retval None
  */
void FlightController_Init(float dt)
{
  flight_controller_dt = dt;
  // Initialize PID controllers with example gains
  PID_Init(&pid_roll, 0.5f, 0.01f, 0.2f, 400.0f);  // Adjust Kp, Ki, Kd and limit as needed
  PID_Init(&pid_pitch, 0.5f, 0.01f, 0.2f, 400.0f);
  PID_Init(&pid_yaw, 1.0f, 0.05f, 0.5f, 400.0f);

  // Initialize sensor fusion
  ComplementaryFilter_Init(dt, 0.98f); // Adjust gyro_weight as needed
}

/**
  * @brief  Runs the flight controller loop (angle/self-level mode).
  * @param  rc_channels: Pointer to ELRS_RC_Channels_t containing remote control data.
  * @param  current_attitude: Pointer to Attitude_t containing current estimated attitude.
  * @retval None
  */
void FlightController_Run(ELRS_RC_Channels_t* rc_channels, Attitude_t* current_attitude)
{
  // Convert RC input to desired setpoints
  // Assuming RC channels are mapped from 1000-2000, centered at 1500.
  // Need to scale these to desired angle ranges (e.g., -30 to +30 degrees for roll/pitch)
  float target_roll = (rc_channels->roll - 1500) * 0.06f;  // Example: -500 to 500 -> -30 to 30 degrees
  float target_pitch = (rc_channels->pitch - 1500) * 0.06f;
  float target_yaw_rate = (rc_channels->yaw - 1500) * 0.1f; // Example: target yaw rate

  // Calculate PID outputs
  float roll_output = PID_Update(&pid_roll, target_roll, current_attitude->roll, flight_controller_dt);
  float pitch_output = PID_Update(&pid_pitch, target_pitch, current_attitude->pitch, flight_controller_dt);
  // For yaw, we control yaw rate, so setpoint is target_yaw_rate, measured is current_gyro_yaw_rate
  // For simplicity, using attitude->yaw directly for now. In a real system, you'd use gyro_z for yaw rate control.
  float yaw_output = PID_Update(&pid_yaw, target_yaw_rate, 0, flight_controller_dt); // Placeholder, needs actual gyro_z for measured_value

  // Mix PID outputs to motor commands
  // This is a simplified mixer for a quadcopter (X-configuration assumed)
  float motor_fr = rc_channels->throttle + pitch_output - roll_output + yaw_output; // Front-Right
  float motor_fl = rc_channels->throttle + pitch_output + roll_output - yaw_output; // Front-Left
  float motor_rr = rc_channels->throttle - pitch_output - roll_output - yaw_output; // Rear-Right
  float motor_rl = rc_channels->throttle - pitch_output + roll_output + yaw_output; // Rear-Left

  // Clamp motor outputs to valid Dshot range (e.g., 0-2000)
  motor_fr = fmaxf(0, fminf(2000, motor_fr));
  motor_fl = fmaxf(0, fminf(2000, motor_fl));
  motor_rr = fmaxf(0, fminf(2000, motor_rr));
  motor_rl = fmaxf(0, fminf(2000, motor_rl));

  // Send Dshot commands to motors
  Dshot_SetMotorSpeed(0, (uint16_t)motor_fr);
  Dshot_SetMotorSpeed(1, (uint16_t)motor_fl);
  Dshot_SetMotorSpeed(2, (uint16_t)motor_rr);
  Dshot_SetMotorSpeed(3, (uint16_t)motor_rl);
}

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static SPI_HandleTypeDef* hspi_bmi270;
static I2C_HandleTypeDef* hi2c_bmm150;
static SPI_HandleTypeDef* hspi_sx1281;
static TIM_HandleTypeDef* htim_motor[4];
static Attitude_t current_attitude;
static float complementary_filter_dt;
static float complementary_filter_gyro_weight;
static PID_Controller_t pid_roll, pid_pitch, pid_yaw;
static float flight_controller_dt;

/* USER CODE END PV */

/* USER CODE BEGIN 0 */

/**
  * @brief  Initializes the SPI handle for BMI270 communication.
  * @param  hspi: Pointer to the SPI_HandleTypeDef structure.
  * @retval None
  */
void BMI270_SPI_Init(SPI_HandleTypeDef* hspi)
{
  hspi_bmi270 = hspi;
  // Configure CS pin as output and set high (inactive)
  HAL_GPIO_WritePin(BMI270_CS_GPIO_Port, BMI270_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Controls the Chip Select (CS) pin for BMI270.
  * @param  enable: 0 to disable (CS high), 1 to enable (CS low).
  * @retval None
  */
void BMI270_ChipSelect(uint8_t enable)
{
  if (enable)
  {
    HAL_GPIO_WritePin(BMI270_CS_GPIO_Port, BMI270_CS_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(BMI270_CS_GPIO_Port, BMI270_CS_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  Reads a register from BMI270 via SPI.
  * @param  reg_addr: Address of the register to read.
  * @retval Value read from the register.
  */
uint8_t BMI270_ReadRegister(uint8_t reg_addr)
{
  uint8_t tx_data[2];
  uint8_t rx_data[2];

  tx_data[0] = reg_addr | 0x80; // Set MSB for read operation
  tx_data[1] = 0x00;           // Dummy byte for reading

  BMI270_ChipSelect(1); // Enable CS
  HAL_SPI_TransmitReceive(hspi_bmi270, tx_data, rx_data, 2, HAL_MAX_DELAY);
  BMI270_ChipSelect(0); // Disable CS

  return rx_data[1];
}

/**
  * @brief  Writes a value to a register in BMI270 via SPI.
  * @param  reg_addr: Address of the register to write.
  * @param  value: Value to write to the register.
  * @retval None
  */
void BMI270_WriteRegister(uint8_t reg_addr, uint8_t value)
{
  uint8_t tx_data[2];

  tx_data[0] = reg_addr & 0x7F; // Clear MSB for write operation
  tx_data[1] = value;

  BMI270_ChipSelect(1); // Enable CS
  HAL_SPI_Transmit(hspi_bmi270, tx_data, 2, HAL_MAX_DELAY);
  BMI270_ChipSelect(0); // Disable CS
}

/**
  * @brief  Initializes the I2C handle for BMM150 communication.
  * @param  hi2c: Pointer to the I2C_HandleTypeDef structure.
  * @retval None
  */
void BMM150_I2C_Init(I2C_HandleTypeDef* hi2c)
{
  hi2c_bmm150 = hi2c;
}

/**
  * @brief  Reads a register from BMM150 via I2C.
  * @param  reg_addr: Address of the register to read.
  * @retval Value read from the register.
  */
uint8_t BMM150_ReadRegister(uint8_t reg_addr)
{
  uint8_t rx_data;
  HAL_I2C_Mem_Read(hi2c_bmm150, BMM150_I2C_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, &rx_data, 1, HAL_MAX_DELAY);
  return rx_data;
}

/**
  * @brief  Writes a value to a register in BMM150 via I2C.
  * @param  reg_addr: Address of the register to write.
  * @param  value: Value to write to the register.
  * @retval None
  */
void BMM150_WriteRegister(uint8_t reg_addr, uint8_t value)
{
  HAL_I2C_Mem_Write(hi2c_bmm150, BMM150_I2C_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Initializes the BMI270 sensor.
  * @retval 0 on failure, 1 on success.
  */
uint8_t BMI270_Init(void)
{
  uint8_t chip_id = BMI270_ReadRegister(BMI270_CHIP_ID_REG);
  if (chip_id != 0x24) // BMI270 Chip ID is 0x24
  {
    return 0; // Initialization failed
  }

  // Soft reset
  BMI270_WriteRegister(BMI270_CMD_REG, BMI270_SOFT_RESET_CMD);
  HAL_Delay(10); // Wait for reset to complete

  // Enable accelerometer and gyroscope
  BMI270_WriteRegister(BMI270_ACC_CONF_REG, BMI270_ACC_ENABLE_CMD); // Example: Enable ACC
  BMI270_WriteRegister(BMI270_GYR_CONF_REG, BMI270_GYR_ENABLE_CMD); // Example: Enable GYR

  return 1; // Initialization successful
}

/**
  * @brief  Reads accelerometer and gyroscope data from BMI270.
  * @param  data: Pointer to the BMI270_SensorData_t structure to store data.
  * @retval None
  */
void BMI270_ReadSensorData(BMI270_SensorData_t* data)
{
  uint8_t raw_data[12];

  // Read accelerometer data (6 bytes starting from 0x0C for X, Y, Z)
  // Read gyroscope data (6 bytes starting from 0x12 for X, Y, Z)

  // Example for reading data (adjust register addresses based on BMI270 datasheet)
  // Assuming Accel X LSB at 0x0C, MSB at 0x0D, etc.
  // Assuming Gyro X LSB at 0x12, MSB at 0x13, etc.

  // For simplicity, let's assume we read 12 bytes starting from 0x0C (Acc_X_LSB)
  // NOTE: You need to consult BMI270 datasheet for exact register addresses and read sequence.
  // This is a placeholder and needs actual implementation based on datasheet.

  // Placeholder for reading multiple bytes (replace with actual burst read if available or sequential reads)
  BMI270_ChipSelect(1); // Enable CS
  uint8_t tx_start_addr = 0x0C | 0x80; // Start from ACC_X_LSB
  HAL_SPI_Transmit(hspi_bmi270, &tx_start_addr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(hspi_bmi270, raw_data, 12, HAL_MAX_DELAY);
  BMI270_ChipSelect(0); // Disable CS

  data->acc_x = (int16_t)(raw_data[1] << 8 | raw_data[0]);
  data->acc_y = (int16_t)(raw_data[3] << 8 | raw_data[2]);
  data->acc_z = (int16_t)(raw_data[5] << 8 | raw_data[4]);

  data->gyr_x = (int16_t)(raw_data[7] << 8 | raw_data[6]);
  data->gyr_y = (int16_t)(raw_data[9] << 8 | raw_data[8]);
  data->gyr_z = (int16_t)(raw_data[11] << 8 | raw_data[10]);
}

/**
  * @brief  Initializes the BMM150 sensor.
  * @retval 0 on failure, 1 on success.
  */
uint8_t BMM150_Init(void)
{
  uint8_t chip_id = BMM150_ReadRegister(BMM150_CHIP_ID_REG);
  if (chip_id != 0x32) // BMM150 Chip ID is 0x32
  {
    return 0; // Initialization failed
  }

  // Soft reset (if available, consult datasheet)
  // BMM150_WriteRegister(BMM150_CMD_REG, BMM150_SOFT_RESET_CMD);
  // HAL_Delay(10);

  // Enable power control and set normal mode
  BMM150_WriteRegister(BMM150_POWER_CTRL_REG, BMM150_POWER_CTRL_ENABLE);
  BMM150_WriteRegister(BMM150_OP_MODE_REG, BMM150_OP_MODE_NORMAL);

  return 1; // Initialization successful
}

/**
  * @brief  Reads magnetometer data from BMM150.
  * @param  data: Pointer to the BMM150_SensorData_t structure to store data.
  * @retval None
  */
void BMM150_ReadSensorData(BMM150_SensorData_t* data)
{
  uint8_t raw_data[6];

  // Read magnetometer data (6 bytes starting from BMM150_DATA_X_LSB_REG)
  HAL_I2C_Mem_Read(hi2c_bmm150, BMM150_I2C_ADDRESS, BMM150_DATA_X_LSB_REG, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY);

  // Combine LSB and MSB to get 16-bit values
  // Consult BMM150 datasheet for correct byte order and data format
  data->mag_x = (int16_t)(raw_data[1] << 8 | raw_data[0]);
  data->mag_y = (int16_t)(raw_data[3] << 8 | raw_data[2]);
  data->mag_z = (int16_t)(raw_data[5] << 8 | raw_data[4]);
}

/**
  * @brief  Initializes the SPI handle for SX1281 communication.
  * @param  hspi: Pointer to the SPI_HandleTypeDef structure.
  * @retval None
  */
void SX1281_SPI_Init(SPI_HandleTypeDef* hspi)
{
  hspi_sx1281 = hspi;
  HAL_GPIO_WritePin(SX1281_CS_GPIO_Port, SX1281_CS_Pin, GPIO_PIN_SET); // CS inactive
}

/**
  * @brief  Controls the Chip Select (CS) pin for SX1281.
  * @param  enable: 0 to disable (CS high), 1 to enable (CS low).
  * @retval None
  */
void SX1281_ChipSelect(uint8_t enable)
{
  if (enable)
  {
    HAL_GPIO_WritePin(SX1281_CS_GPIO_Port, SX1281_CS_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(SX1281_CS_GPIO_Port, SX1281_CS_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  Resets the SX1281 module.
  * @retval None
  */
void SX1281_Reset(void)
{
  HAL_GPIO_WritePin(SX1281_RESET_GPIO_Port, SX1281_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1); // Datasheet recommends 100us minimum reset pulse
  HAL_GPIO_WritePin(SX1281_RESET_GPIO_Port, SX1281_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(6); // Wait for chip to be ready
}

/**
  * @brief  Reads a register from SX1281 via SPI.
  * @param  reg_addr: Address of the register to read.
  * @retval Value read from the register.
  */
uint8_t SX1281_ReadRegister(uint16_t reg_addr)
{
  uint8_t tx_data[4];
  uint8_t rx_data[4];

  tx_data[0] = SX1281_READ_REGISTER_CMD; // Command
  tx_data[1] = (reg_addr >> 8) & 0xFF;    // Address MSB
  tx_data[2] = reg_addr & 0xFF;           // Address LSB
  tx_data[3] = 0x00;                     // Dummy byte for reading

  SX1281_ChipSelect(1);
  HAL_SPI_TransmitReceive(hspi_sx1281, tx_data, rx_data, 4, HAL_MAX_DELAY);
  SX1281_ChipSelect(0);

  return rx_data[3];
}

/**
  * @brief  Writes a value to a register in SX1281 via SPI.
  * @param  reg_addr: Address of the register to write.
  * @param  value: Value to write to the register.
  * @retval None
  */
void SX1281_WriteRegister(uint16_t reg_addr, uint8_t value)
{
  uint8_t tx_data[4];

  tx_data[0] = SX1281_WRITE_REGISTER_CMD; // Command
  tx_data[1] = (reg_addr >> 8) & 0xFF;    // Address MSB
  tx_data[2] = reg_addr & 0xFF;           // Address LSB
  tx_data[3] = value;                     // Value to write

  SX1281_ChipSelect(1);
  HAL_SPI_Transmit(hspi_sx1281, tx_data, 4, HAL_MAX_DELAY);
  SX1281_ChipSelect(0);
}

/**
  * @brief  Writes a command to SX1281 via SPI.
  * @param  cmd: Command to write.
  * @retval None
  */
void SX1281_WriteCommand(uint8_t cmd)
{
  SX1281_ChipSelect(1);
  HAL_SPI_Transmit(hspi_sx1281, &cmd, 1, HAL_MAX_DELAY);
  SX1281_ChipSelect(0);
}

/**
  * @brief  Reads data from SX1281 buffer via SPI.
  * @param  offset: Offset in the buffer to start reading.
  * @param  buffer: Pointer to the buffer to store received data.
  * @param  size: Number of bytes to read.
  * @retval None
  */
void SX1281_ReadBuffer(uint8_t offset, uint8_t* buffer, uint8_t size)
{
  uint8_t tx_header[2] = {0x1C, offset}; // Read buffer command + offset

  SX1281_ChipSelect(1);
  HAL_SPI_Transmit(hspi_sx1281, tx_header, 2, HAL_MAX_DELAY);
  HAL_SPI_Receive(hspi_sx1281, buffer, size, HAL_MAX_DELAY);
  SX1281_ChipSelect(0);
}

/**
  * @brief  Initializes the ELRS receiver (SX1281).
  * @retval 0 on failure, 1 on success.
  */
uint8_t ELRS_Init(void)
{
  SX1281_Reset();
  // Basic configuration for SX1281 for ELRS-like operation
  // This is a placeholder and needs actual SX1281 register configuration for ELRS.
  SX1281_WriteCommand(SX1281_SET_STANDBY_CMD); // Example: set to standby mode
  // Further configuration like frequency, packet type, etc., would go here.
  return 1;
}

/**
  * @brief  Receives and parses an ELRS packet.
  * @param  channels: Pointer to the ELRS_RC_Channels_t structure to store channel data.
  * @retval 0 on no new packet, 1 on successful packet reception and parsing.
  */
uint8_t ELRS_ReceivePacket(ELRS_RC_Channels_t* channels)
{
  // This is a highly simplified placeholder. A real ELRS implementation
  // would involve checking for received packets, CRC, decoding, etc.
  // For demonstration, let's simulate receiving some data.

  uint8_t packet_data[16]; // Example packet size
  // Assume SX1281 has received a packet and it's in its internal buffer.
  // You would typically read status registers to check for new packets.

  // For now, let's just read a dummy buffer.
  SX1281_ReadBuffer(0, packet_data, 16); // Read 16 bytes from buffer offset 0

  // Simulate channel data from the received packet
  // This mapping is purely illustrative and needs to match actual ELRS packet format.
  channels->roll = (uint16_t)(packet_data[0] << 8 | packet_data[1]);
  channels->pitch = (uint16_t)(packet_data[2] << 8 | packet_data[3]);
  channels->yaw = (uint16_t)(packet_data[4] << 8 | packet_data[5]);
  channels->throttle = (uint16_t)(packet_data[6] << 8 | packet_data[7]);
  channels->aux1 = (uint16_t)(packet_data[8] << 8 | packet_data[9]);
  channels->aux2 = (uint16_t)(packet_data[10] << 8 | packet_data[11]);

  return 1; // Assume packet received and parsed successfully for now
}

/**
  * @brief  Initializes the timers for Dshot output.
  * @param  htim_ch1: Pointer to TIM_HandleTypeDef for motor 1.
  * @param  htim_ch2: Pointer to TIM_HandleTypeDef for motor 2.
  * @param  htim_ch3: Pointer to TIM_HandleTypeDef for motor 3.
  * @param  htim_ch4: Pointer to TIM_HandleTypeDef for motor 4.
  * @retval None
  */
void Dshot_Timer_Init(TIM_HandleTypeDef* htim_ch1, TIM_HandleTypeDef* htim_ch2, TIM_HandleTypeDef* htim_ch3, TIM_HandleTypeDef* htim_ch4)
{
  htim_motor[0] = htim_ch1;
  htim_motor[1] = htim_ch2;
  htim_motor[2] = htim_ch3;
  htim_motor[3] = htim_ch4;

  // Start PWM for all channels
  HAL_TIM_PWM_Start(htim_motor[0], TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim_motor[1], TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(htim_motor[2], TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim_motor[3], TIM_CHANNEL_4);

  // Configure timer period and prescaler based on Dshot speed
  // The actual configuration (ARR, PSC) depends on the specific timer and desired Dshot speed.
  // This will typically be done in CubeIDE, but here's a conceptual setup.

  // Example for DSHOT300 with 72MHz clock (adjust as needed):
  // Timer Clock = 72MHz
  // DSHOT300 period = 3.33us (1/300kHz)
  // If TIM_CLOCK = 72MHz, then 1 tick = 1/72MHz = 0.0138us
  // To get 3.33us period, ARR = 3.33us / 0.0138us = 241 (approx)
  // High pulse for '0' (0.83us) -> CCR = 0.83us / 0.0138us = 60
  // High pulse for '1' (1.67us) -> CCR = 1.67us / 0.0138us = 120

  // For Dshot, we set the timer in One Pulse Mode or manually manipulate CCR.
  // Here we use a simpler approach for illustration, assuming PWM generation.
}

/**
  * @brief  Sets the motor speed using Dshot protocol.
  * @param  motor_id: ID of the motor (0-3).
  * @param  throttle: Throttle value (0-2000, 0 for disarmed, 1-47 for bidirectional reverse, 48-2047 for forward).
  * @retval None
  */
void Dshot_SetMotorSpeed(uint8_t motor_id, uint16_t throttle)
{
  if (motor_id >= 4) return; // Invalid motor ID

  uint32_t dshot_frame[DSHOT_DMA_BUFFER_SIZE];
  Dshot_GenerateFrame(throttle, dshot_frame);

  // Here, you would typically start a DMA transfer to send dshot_frame to the timer's CCR register.
  // For this example, we'll just set the first CCR for demonstration.
  switch (motor_id)
  {
    case 0:
      // Assuming TIM1_CH1 is used for Motor 0
      // This is a placeholder for DMA setup
      HAL_TIM_PWM_Start(htim_motor[0], TIM_CHANNEL_1);
      htim_motor[0]->Instance->CCR1 = dshot_frame[0]; // Just setting first bit for illustration
      break;
    case 1:
      // Assuming TIM1_CH2 is used for Motor 1
      HAL_TIM_PWM_Start(htim_motor[1], TIM_CHANNEL_2);
      htim_motor[1]->Instance->CCR2 = dshot_frame[0];
      break;
    case 2:
      // Assuming TIM1_CH3 is used for Motor 2
      HAL_TIM_PWM_Start(htim_motor[2], TIM_CHANNEL_3);
      htim_motor[2]->Instance->CCR3 = dshot_frame[0];
      break;
    case 3:
      // Assuming TIM1_CH4 is used for Motor 3
      HAL_TIM_PWM_Start(htim_motor[3], TIM_CHANNEL_4);
      htim_motor[3]->Instance->CCR4 = dshot_frame[0];
      break;
  }
}

/**
  * @brief  Generates a Dshot frame for a given throttle value.
  * @param  throttle_value: Throttle value (0-2000).
  * @param  dshot_frame_buffer: Pointer to the buffer to store the generated Dshot frame (CCR values).
  * @retval None
  */
void Dshot_GenerateFrame(uint16_t throttle_value, uint32_t* dshot_frame_buffer)
{
  uint16_t dshot_command = throttle_value << 1; // Shift by 1 for telemetry bit (0 for no telemetry)

  // Calculate CRC
  uint8_t crc = 0;
  crc ^= (dshot_command >> 8);
  crc ^= (dshot_command >> 4);
  crc ^= (dshot_command);
  crc &= 0x0F;
  dshot_command = (dshot_command << 4) | crc;

  // Populate the dshot_frame_buffer with appropriate CCR values
  for (int i = 0; i < DSHOT_FRAME_LENGTH; i++)
  {
    if ((dshot_command >> (DSHOT_FRAME_LENGTH - 1 - i)) & 0x01)
    {
      // Bit is '1'
      dshot_frame_buffer[i] = DSHOT_BIT_1_HIGH_TIME; // High pulse for '1'
    }
    else
    {
      // Bit is '0'
      dshot_frame_buffer[i] = DSHOT_BIT_0_HIGH_TIME; // High pulse for '0'
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  Flight Controller Initialization Function
  * @param  None
  * @retval None
  */
void FlightController_Init(void)
{
    // 初始化PID控制器
    pidRoll.kp = PID_ROLL_KP;
    pidRoll.ki = PID_ROLL_KI;
    pidRoll.kd = PID_ROLL_KD;
    pidRoll.prev_error = 0.0f;
    pidRoll.integral = 0.0f;
    pidRoll.output_limit = 400.0f; // 假设电机输出范围0-1000
    pidRoll.integral_limit = 200.0f;

    pidPitch.kp = PID_PITCH_KP;
    pidPitch.ki = PID_PITCH_KI;
    pidPitch.kd = PID_PITCH_KD;
    pidPitch.prev_error = 0.0f;
    pidPitch.integral = 0.0f;
    pidPitch.output_limit = 400.0f;
    pidPitch.integral_limit = 200.0f;

    pidYaw.kp = PID_YAW_KP;
    pidYaw.ki = PID_YAW_KI;
    pidYaw.kd = PID_YAW_KD;
    pidYaw.prev_error = 0.0f;
    pidYaw.integral = 0.0f;
    pidYaw.output_limit = 400.0f;
    pidYaw.integral_limit = 200.0f;
}

/**
  * @brief  Flight Controller Main Loop Function
  * @param  None
  * @retval None
  */
void FlightController_Loop(void)
{
    SensorRawData_t rawData;
    SensorCalibratedData_t calibratedData;
    Attitude_t currentAttitude;
    uint16_t rc_channels[16]; // 假设16个通道
    float motor_outputs[4]; // 假设4个电机
    float battery_voltage; // 电池电压变量

    // 1. 读取传感器原始数据
    if (FC_ReadSensors(&rawData) == HAL_OK)
    {
        // 2. 校准传感器数据
        FC_CalibrateSensors(&rawData, &calibratedData);

        // 3. 姿态估计
        FC_EstimateAttitude(&calibratedData, &currentAttitude);

        // 4. 读取遥控器数据 (此处简化，实际需要从SX1281获取)
        // ELRS_Get_Channels(&hspi1, SX1281_CS_GPIO_Port, SX1281_CS_Pin, SX1281_BUSY_GPIO_Port, SX1281_BUSY_Pin, rc_channels);
        // 模拟遥控器输入
        rc_channels[0] = 1500; // 油门
        rc_channels[1] = 1500; // 横滚
        rc_channels[2] = 1500; // 俯仰
        rc_channels[3] = 1500; // 偏航

        // 5. 读取电池电压
        if (FC_ReadBatteryVoltage(&battery_voltage) == HAL_OK)
        {
            // 可以在这里使用电池电压，例如进行低电压报警
            // printf("Battery Voltage: %.2fV\n", battery_voltage);
        }

        // 6. 飞行控制循环
        FC_ControlLoop(&currentAttitude, rc_channels, currentFlightMode, motor_outputs);

        // 在这里可以使用motor_outputs来控制DShot电机
    }
}

/**
  * @brief  读取传感器原始数据
  * @param  rawData: 指向SensorRawData_t结构体的指针，用于存储原始数据
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef FC_ReadSensors(SensorRawData_t *rawData)
{
    HAL_StatusTypeDef status_acc_gyro, status_mag;

    // 读取BMI270 (加速度计和陀螺仪)
    status_acc_gyro = BMI270_ReadAccGyro(&hspi1, BMI270_CS_GPIO_Port, BMI270_CS_Pin, &rawData->acc_x, &rawData->acc_y, &rawData->acc_z, &rawData->gyro_x, &rawData->gyro_y, &rawData->gyro_z);

    // 读取BMM150 (磁力计)
    status_mag = BMM150_ReadMag(&hi2c1, &rawData->mag_x, &rawData->mag_y, &rawData->mag_z);

    if (status_acc_gyro == HAL_OK && status_mag == HAL_OK)
    {
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}

/**
  * @brief  校准传感器数据
  * @param  rawData: 原始传感器数据
  * @param  calibratedData: 校准后的传感器数据
  * @retval None
  */
void FC_CalibrateSensors(SensorRawData_t *rawData, SensorCalibratedData_t *calibratedData)
{
    // TODO: 实现传感器校准算法
    // 这包括陀螺仪零偏校准，加速度计零偏和比例因子校准，磁力计硬铁/软铁校准
    // 目前先将原始数据直接赋值给校准数据
    calibratedData->acc_x = rawData->acc_x / 16384.0f; // 假设量程为+/-2g，灵敏度为16384 LSB/g
    calibratedData->acc_y = rawData->acc_y / 16384.0f;
    calibratedData->acc_z = rawData->acc_z / 16384.0f;

    calibratedData->gyro_x = rawData->gyro_x / 131.0f; // 假设量程为+/-2000dps，灵敏度为131 LSB/dps
    calibratedData->gyro_y = rawData->gyro_y / 131.0f;
    calibratedData->gyro_z = rawData->gyro_z / 131.0f;

    calibratedData->mag_x = rawData->mag_x * 0.1f; // 假设灵敏度为0.1uT/LSB
    calibratedData->mag_y = rawData->mag_y * 0.1f;
    calibratedData->mag_z = rawData->mag_z * 0.1f;

}

/**
  * @brief  姿态估计
  * @param  calibratedData: 校准后的传感器数据
  * @param  attitude: 估计出的姿态角
  * @retval None
  */
void FC_EstimateAttitude(SensorCalibratedData_t *calibratedData, Attitude_t *attitude)
{
    // TODO: 实现姿态融合算法 (例如互补滤波或卡尔曼滤波)
    // 目前先将姿态角设置为0
    attitude->roll = 0.0f;
    attitude->pitch = 0.0f;
    attitude->yaw = 0.0f;
}

/**
  * @brief  PID控制器计算
  * @param  pid: PID控制器结构体
  * @param  setpoint: 目标值
  * @param  measured: 测量值
  * @param  dt: 时间间隔
  * @retval PID输出
  */
float PID_Calculate(PID_Controller_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    pid->integral += error * dt;
    // 积分限幅
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

/**
  * @brief  飞行控制循环
  * @param  currentAttitude: 当前姿态
  * @param  rc_channels: 遥控器通道数据
  * @param  flightMode: 飞行模式
  * @param  motor_outputs: 电机输出 (0-1000)
  * @retval None
  */
void FC_ControlLoop(Attitude_t *currentAttitude, uint16_t *rc_channels, FlightMode_t flightMode, float *motor_outputs)
{
    float targetRoll = 0.0f;
    float targetPitch = 0.0f;
    float targetYawRate = 0.0f;
    float throttle = 0.0f;

    // 从遥控器通道获取目标值
    throttle = (float)(rc_channels[0] - 1000) / 1000.0f; // 油门 0-1000

    // 根据飞行模式设置目标姿态或角速率
    if (flightMode == MODE_ANGLE) // 自稳模式
    {
        targetRoll = (float)(rc_channels[1] - 1500) / 500.0f * 30.0f; // 横滚角度目标 +/-30度
        targetPitch = (float)(rc_channels[2] - 1500) / 500.0f * 30.0f; // 俯仰角度目标 +/-30度
        targetYawRate = (float)(rc_channels[3] - 1500) / 500.0f * 200.0f; // 偏航角速率目标 +/-200度/秒
    }
    else if (flightMode == MODE_ACRO) // 角度模式 (角速率控制)
    {
        targetRoll = (float)(rc_channels[1] - 1500) / 500.0f * 200.0f; // 横滚角速率目标 +/-200度/秒
        targetPitch = (float)(rc_channels[2] - 1500) / 500.0f * 200.0f; // 俯仰角速率目标 +/-200度/秒
        targetYawRate = (float)(rc_channels[3] - 1500) / 500.0f * 200.0f; // 偏航角速率目标 +/-200度/秒
    }

    // 计算PID输出
    float pid_roll_output = PID_Calculate(&pidRoll, targetRoll, currentAttitude->roll, 0.004f); // 假设控制周期为4ms
    float pid_pitch_output = PID_Calculate(&pidPitch, targetPitch, currentAttitude->pitch, 0.004f);
    float pid_yaw_output = PID_Calculate(&pidYaw, targetYawRate, calibratedData.gyro_z, 0.004f); // 偏航控制通常是角速率控制

    // 将PID输出映射到电机输出 (四旋翼混合算法)
    // 这里是一个简化的混合算法，实际需要更复杂的模型
    motor_outputs[0] = throttle * 1000.0f + pid_pitch_output - pid_roll_output - pid_yaw_output; // 前右
    motor_outputs[1] = throttle * 1000.0f - pid_pitch_output - pid_roll_output + pid_yaw_output; // 后右
    motor_outputs[2] = throttle * 1000.0f - pid_pitch_output + pid_roll_output - pid_yaw_output; // 前左
    motor_outputs[3] = throttle * 1000.0f + pid_pitch_output + pid_roll_output + pid_yaw_output; // 后左

    // 电机输出限幅 (0-1000)
    for (int i = 0; i < 4; i++) {
        if (motor_outputs[i] > 1000.0f) motor_outputs[i] = 1000.0f;
        if (motor_outputs[i] < 0.0f) motor_outputs[i] = 0.0f;

        // 调用DShot_SetThrottle发送电机指令
        Dshot_SetThrottle(i, (uint16_t)motor_outputs[i]);
    }
}

/**
  * @brief  读取电池电压
  * @param  voltage: 指向float变量的指针，用于存储电池电压
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef FC_ReadBatteryVoltage(float *voltage)
{
    HAL_StatusTypeDef status;
    uint32_t adc_raw_value;

    // 启动ADC转换
    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK) return status;

    // 等待ADC转换完成
    status = HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // 获取ADC原始值
    adc_raw_value = HAL_ADC_GetValue(&hadc1);

    // 停止ADC转换
    status = HAL_ADC_Stop(&hadc1);
    if (status != HAL_OK) return status;

    // 将ADC原始值转换为电压 (假设ADC分辨率为12位，参考电压为3.3V)
    // ADC_MAX_VALUE = 2^12 - 1 = 4095
    *voltage = (float)adc_raw_value / 4095.0f * 3.3f * VOLTAGE_DIVIDER_RATIO;

    return HAL_OK;
}

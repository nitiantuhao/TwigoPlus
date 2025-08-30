#include "Balance/balance_control.h"
#include "gpio.h"
#include "tim.h"
#include "encoder.h"
#include <math.h>

/* ---------- 全局变量 ---------- */
PID_HandleTypeDef balance_pid;
PID_HandleTypeDef speed_pid;

float current_pitch = 0.0f;
uint8_t data_ready = 0;
float target_speed = 0.0f;          // 期望线速度（cm/s）
float target_yaw = 0.0f;
int16_t encoder_speed_left = 0;
int16_t encoder_speed_right = 0;

/* ---------- 宏/常量 ---------- */
#define MIN_START_PWM  1.0f
#define SPEED_K        8.18f        // 编码器计数→cm/s 的系数，按轮子/减速比实际标定
#define SPEED_T        0.001f       // 1 ms
#define SPEED_LPF_ALPHA 0.3f
/* ---------- 静态变量 ---------- */
static int32_t last_left  = 0;
static int32_t last_right = 0;
static float   measured_speed = 0.0f;

/* ---------- 中断/回调 ---------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_14) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        float roll, yaw;
        static float last_valid_pitch = 0.0f;
        if (MPU6050_DMP_Get_Date(&current_pitch, &roll, &yaw) == 0) {
            if (fabs(current_pitch - last_valid_pitch) < 5.0f) {
                last_valid_pitch = current_pitch;
                data_ready = 1;
            }
        }
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
    }
}

/* TIM3 1 ms 中断：计算速度 */
static float speed_lpf = 0.0f;     // 滤波后的速度

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    /* 1. 读编码器差分 */
    int32_t left  = Encoder_Get_Count(ENCODER_LEFT);
    int32_t right = Encoder_Get_Count(ENCODER_RIGHT);

    int32_t delta_left  = left  - last_left;
    int32_t delta_right = right - last_right;

    last_left  = left;
    last_right = right;

    /* 2. 先算瞬时速度（cm/s） */
    float raw_speed = (float)(delta_left + delta_right) * 0.5f * SPEED_K / SPEED_T;

    /* 3. 一阶低通滤波 */
    speed_lpf = SPEED_LPF_ALPHA * speed_lpf + (1.0f - SPEED_LPF_ALPHA) * raw_speed;
  }
}

/* ---------- 电机阈值 ---------- */
float Motor_Start_Threshold(float pwm) {
    if (fabs(pwm) > 0 && fabs(pwm) < MIN_START_PWM)
        return (pwm > 0) ? MIN_START_PWM : -MIN_START_PWM;
    return pwm;
}

/* ---------- PID 初始化 ---------- */
void PID_Init(void) {
    /* 角度环 */
    balance_pid.kp = 6.0f;
    balance_pid.ki = 0.03f;
    balance_pid.kd = 0.6f;
    balance_pid.target = 8.0f;          // 机械平衡点
    balance_pid.error = 0.0f;
    balance_pid.last_err = 0.0f;
    balance_pid.integral = 0.0f;
    balance_pid.max_out = 100.0f;
    balance_pid.min_out = -100.0f;
    balance_pid.Ts = 0.01f;             // 10 ms
    balance_pid.alpha = 0.7f;
    balance_pid.deadband = 0.5f;
    balance_pid.last_current = 0.0f;
    balance_pid.diff_filtered = 0.0f;

    /* 速度环 */
    speed_pid.kp = 20.0f;               // 先设 20~40，现场调
    speed_pid.ki = 0.05f;
    speed_pid.kd = 0.60f;
    speed_pid.target = 0.0f;            // 静止
    speed_pid.error = 0.0f;
    speed_pid.last_err = 0.0f;
    speed_pid.integral = 0.0f;
    speed_pid.max_out = 8.0f;           // 输出：角度增量
    speed_pid.min_out = -8.0f;
    speed_pid.Ts = 0.01f;               // 10 ms
    speed_pid.alpha = 0.0f;
    speed_pid.deadband = 0.2f;
    speed_pid.last_current = 0.0f;
    speed_pid.diff_filtered = 0.0f;
}

/* ---------- 初始化 ---------- */
void MPU6050_Interrupt_Init(void) {
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
}

void Balance_Init(void) {
    TB6612_Init();
    while (MPU6050_DMP_init() != 0) HAL_Delay(500);

    Encoder_Init();
    PID_Init();
    MPU6050_Interrupt_Init();

    /* TIM3 1 kHz 中断，CubeMX 已配好 */
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_Delay(1000);
}

/* ---------- PID 计算 ---------- */
float PID_Calculate(PID_HandleTypeDef *pid, float current) {
    pid->error = pid->target - current;
    if (fabs(pid->error) < pid->deadband) pid->error = 0.0f;

    if (fabs(pid->error) < 5.0f) {
        if (pid->output < pid->max_out && pid->output > pid->min_out)
            pid->integral += pid->error * pid->Ts;

        float max_i = pid->max_out / pid->ki;
        float min_i = pid->min_out / pid->ki;
        if (pid->integral > max_i) pid->integral = max_i;
        if (pid->integral < min_i) pid->integral = min_i;
    } else {
        pid->integral = 0;
    }

    float current_diff = (current - pid->last_current) / pid->Ts;
    pid->diff_filtered = pid->alpha * pid->diff_filtered +
                         (1 - pid->alpha) * (-current_diff);

    pid->output = pid->kp * pid->error +
                  pid->ki * pid->integral +
                  pid->kd * pid->diff_filtered;

    if (pid->output > pid->max_out) pid->output = pid->max_out;
    if (pid->output < pid->min_out) pid->output = pid->min_out;

    pid->last_err = pid->error;
    pid->last_current = current;
    return pid->output;
}

/* ---------- 主控制循环 ---------- */
void Balance_Control(void) {
    if (!data_ready) return;
    data_ready = 0;

    /* 速度环 → 角度补偿 */
    float speed_out = PID_Calculate(&speed_pid, speed_lpf);
    balance_pid.target = 8.0f + speed_out;          // 8° 平衡点

    /* 角度环 → 电机 PWM */
    float balance_output = PID_Calculate(&balance_pid, current_pitch);
    float optimized_output = Motor_Start_Threshold(balance_output);
    int16_t motor_speed = (int16_t)optimized_output;

    if (motor_speed > 0) {
        TB6612_SetDirection(TB6612_MOTOR_A, TB6612_FORWARD);
        TB6612_SetDirection(TB6612_MOTOR_B, TB6612_FORWARD);
        TB6612_SetSpeed(TB6612_MOTOR_A, motor_speed);
        TB6612_SetSpeed(TB6612_MOTOR_B, motor_speed);
    } else if (motor_speed < 0) {
        TB6612_SetDirection(TB6612_MOTOR_A, TB6612_BACKWARD);
        TB6612_SetDirection(TB6612_MOTOR_B, TB6612_BACKWARD);
        TB6612_SetSpeed(TB6612_MOTOR_A, -motor_speed);
        TB6612_SetSpeed(TB6612_MOTOR_B, -motor_speed);
    } else {
        TB6612_HardStop(TB6612_MOTOR_A);
        TB6612_HardStop(TB6612_MOTOR_B);
    }
}
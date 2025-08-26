#include "Balance/balance_control.h"
#include "gpio.h"
#include "tim.h"
#include <math.h>

PID_HandleTypeDef balance_pid;
float current_pitch = 0.0f;
uint8_t data_ready = 0;
PID_HandleTypeDef speed_pid;
PID_HandleTypeDef turn_pid;
float target_speed = 0.0f;
float target_yaw = 0.0f;
int16_t encoder_speed_left = 0;
int16_t encoder_speed_right = 0;

// 新增：电机最小启动PWM阈值（根据实际电机特性调整，通常15-30）
#define MIN_START_PWM 22.0f

// MPU6050中断服务函数（PB14触发）
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_14) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    float roll, yaw;
    // 连续2次读取一致才认为有效（抗突发噪声）
    static float last_valid_pitch = 0.0f;
    if (MPU6050_DMP_Get_Date(&current_pitch, &roll, &yaw) == 0) {
      // 角度突变检测（超过5度认为异常，用上次有效值）
      if (fabs(current_pitch - last_valid_pitch) < 5.0f) {
        last_valid_pitch = current_pitch;
        data_ready = 1;
      }
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
  }
}

// 新增：电机PWM启动阈值处理函数
float Motor_Start_Threshold(float pwm) {
  // 当PWM绝对值大于0但小于启动阈值时，提升到阈值
  if (fabs(pwm) > 0 && fabs(pwm) < MIN_START_PWM) {
    return (pwm > 0) ? MIN_START_PWM : -MIN_START_PWM;
  }
  return pwm; // 其他情况保持原PWM值
}

// PID参数初始化
void PID_Init(void) {
    // 平衡PID参数（需根据实际调试调整）
    balance_pid.kp = 6.0f;    // 比例系数
    balance_pid.ki = 0.03f;    // 积分系数
    balance_pid.kd = 0.6f;    // 微分系数
    balance_pid.target = 8.0f; // 目标角度（平衡位置，需校准）
    balance_pid.error = 0.0f;
    balance_pid.last_err = 0.0f;
    balance_pid.integral = 0.0f;
    balance_pid.max_out = 100.0f;  // 最大输出（小于PWM_MAX=100）
    balance_pid.min_out = -100.0f; // 最小输出
    // 新增参数初始化
    balance_pid.Ts = 0.01f;   // 采样周期（假设MPU中断10ms一次）
    balance_pid.alpha = 0.7f; // 微分滤波系数
    balance_pid.deadband = 0.5f; // 死区0.3度（根据传感器精度调整）
    balance_pid.last_current = 0.0f;
    balance_pid.diff_filtered = 0.0f;

}

// MPU6050中断初始化（PB14）
void MPU6050_Interrupt_Init(void) {
    // 中断已在CubeMX中配置（PB14，下降沿触发）
    // 此处仅使能中断
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
}

// 系统初始化（传感器+电机）
void Balance_Init(void) {
    // 初始化电机
    TB6612_Init();

    // 初始化MPU6050 DMP
    while (MPU6050_DMP_init() != 0) {
        // 初始化失败可添加指示灯提示
        HAL_Delay(500);
    }

    // 初始化中断和PID
    MPU6050_Interrupt_Init();
    PID_Init();

    // 等待传感器稳定
    HAL_Delay(1000);
}

// PID计算函数
float PID_Calculate(PID_HandleTypeDef *pid, float current) {
  // 1. 计算误差（带死区处理，小误差不响应）
  pid->error = pid->target - current;
  if (fabs(pid->error) < pid->deadband) {
    pid->error = 0.0f;
  }

  // 2. 积分项优化（抗积分饱和）
  if (fabs(pid->error) < 5.0f) {  // 积分分离条件不变
    // 仅当输出未达到限幅时累加积分（防止饱和）
    if (pid->output < pid->max_out && pid->output > pid->min_out) {
      // 积分 *= 采样时间，使KI参数与采样频率无关
      pid->integral += pid->error * pid->Ts;
      // 积分限幅（与输出限幅关联，更合理）
      float max_integral = pid->max_out / pid->ki;
      float min_integral = pid->min_out / pid->ki;
      if (pid->integral > max_integral) pid->integral = max_integral;
      if (pid->integral < min_integral) pid->integral = min_integral;
    }
  } else {
    pid->integral = 0;  // 大误差时清零积分
  }

  // 3. 微分项优化（抑制噪声+减少目标突变影响）
  // 用当前值的微分代替误差微分（避免目标突变导致的微分冲击）
  float current_diff = (current - pid->last_current) / pid->Ts;
  // 一阶低通滤波平滑微分（减少传感器噪声）
  pid->diff_filtered = pid->alpha * pid->diff_filtered +
                      (1 - pid->alpha) * (-current_diff);  // 负号：误差微分 = -当前值微分

  // 4. 计算输出（比例+积分+滤波后微分）
  pid->output = pid->kp * pid->error +
               pid->ki * pid->integral +
               pid->kd * pid->diff_filtered;

  // 5. 输出限幅
  if (pid->output > pid->max_out) pid->output = pid->max_out;
  if (pid->output < pid->min_out) pid->output = pid->min_out;

  // 6. 保存历史数据
  pid->last_err = pid->error;
  pid->last_current = current;  // 保存当前值用于下一次微分计算

  return pid->output;
}

// 平衡控制主函数
void Balance_Control(void) {
    if (data_ready) {  // 有新的角度数据时进行控制
        data_ready = 0;  // 清除标志

        // 计算平衡PID输出
        float balance_output = PID_Calculate(&balance_pid, current_pitch);

        // 应用电机启动阈值优化
        float optimized_output = Motor_Start_Threshold(balance_output);
        int16_t motor_speed = (int16_t)optimized_output;

        // 设置电机方向和速度
        if (motor_speed > 0) {
            // 输出为正：小车向前倾，需要后轮向前转维持平衡
            TB6612_SetDirection(TB6612_MOTOR_A, TB6612_FORWARD);
            TB6612_SetDirection(TB6612_MOTOR_B, TB6612_FORWARD);
            TB6612_SetSpeed(TB6612_MOTOR_A, motor_speed);
            TB6612_SetSpeed(TB6612_MOTOR_B, motor_speed);
        } else if (motor_speed < 0) {
            // 输出为负：小车向后倾，需要后轮向后转维持平衡
            TB6612_SetDirection(TB6612_MOTOR_A, TB6612_BACKWARD);
            TB6612_SetDirection(TB6612_MOTOR_B, TB6612_BACKWARD);
            TB6612_SetSpeed(TB6612_MOTOR_A, -motor_speed);
            TB6612_SetSpeed(TB6612_MOTOR_B, -motor_speed);
        } else {
            // 输出为0：停止电机
            TB6612_HardStop(TB6612_MOTOR_A);
            TB6612_HardStop(TB6612_MOTOR_B);
        }
    }
}

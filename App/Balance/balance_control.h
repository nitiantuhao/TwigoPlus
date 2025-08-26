//
// Created by Falling_jasmine on 2025/8/22.
//

#ifndef TWIGO_BALANCE_CONTROL_H
#define TWIGO_BALANCE_CONTROL_H

#include "stm32f1xx_hal.h"
#include "Motor/tb6612.h"
#include "Sensor/mpu6050_dmp.h"
#include "encoder.h"
extern float current_pitch;
// PID参数结构体
typedef struct {
  float kp;       // 比例系数
  float ki;       // 积分系数
  float kd;       // 微分系数
  float target;   // 目标值
  float error;    // 当前误差
  float last_err; // 上一次误差
  float integral; // 积分值
  float output;   // 输出值
  float max_out;  // 最大输出限制
  float min_out;  // 最小输出限制

  // 新增优化参数
  float Ts;               // 采样时间（秒），用于标准化参数
  float last_current;     // 上一次测量值，用于微分计算
  float diff_filtered;    // 滤波后的微分值，抑制噪声
  float alpha;            // 微分滤波系数（0~1，推荐0.6~0.8）
  float deadband;         // 死区阈值（误差小于此值时不响应，减少抖动）
} PID_HandleTypeDef;

// 全局变量声明

extern PID_HandleTypeDef balance_pid;
extern float current_pitch;  // 当前俯仰角
extern uint8_t data_ready;   // 数据就绪标志
extern float target_speed;                // 目标速度
extern float target_yaw;                  // 目标偏航角
extern int16_t encoder_speed_left;        // 左编码器速度
extern int16_t encoder_speed_right;       // 右编码器速度
// 函数声明
void PID_Init(void);
void Balance_Init(void);
float PID_Calculate(PID_HandleTypeDef *pid, float current);
void Balance_Control(void);
void MPU6050_Interrupt_Init(void);

#endif //TWIGO_BALANCE_CONTROL_H
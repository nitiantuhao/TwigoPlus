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

typedef struct {
  float kp;
  float ki;
  float kd;
  float target;
  float error;
  float last_err;
  float integral;
  float output;
  float max_out;
  float min_out;
  float Ts;
  float last_current;
  float diff_filtered;
  float alpha;
  float deadband;
} PID_HandleTypeDef;

extern PID_HandleTypeDef balance_pid;
extern PID_HandleTypeDef speed_pid;   // ← 新增
extern float current_pitch;
extern uint8_t data_ready;
extern float target_speed;
extern float target_yaw;
extern int16_t encoder_speed_left;
extern int16_t encoder_speed_right;

void PID_Init(void);
void Balance_Init(void);
float PID_Calculate(PID_HandleTypeDef *pid, float current);
void Balance_Control(void);
void MPU6050_Interrupt_Init(void);

#endif
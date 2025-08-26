//
// Created by Falling_jasmine on 2025/8/22.
//

#ifndef TWIGO_TB6612_H
#define TWIGO_TB6612_H
#include "pin_definitions.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "tim.h"
extern uint16_t current_speed_a;
extern uint16_t current_speed_b;
// 电机编号
typedef enum {
  TB6612_MOTOR_A,
  TB6612_MOTOR_B
} TB6612_MotorTypeDef;

// 电机方向
typedef enum {
  TB6612_FORWARD,   // 正转
  TB6612_BACKWARD,  // 反转
  TB6612_STOP       // 停止
} TB6612_DirectionTypeDef;

// --------------------------
// 引脚宏定义 - 已按提供的配置修改
// --------------------------
#define TB6612_AIN1_PORT    MOTOR_AIN1_PORT
#define TB6612_AIN1_PIN     MOTOR_AIN1_PIN
#define TB6612_AIN2_PORT    MOTOR_AIN2_PORT
#define TB6612_AIN2_PIN     MOTOR_AIN2_PIN

#define TB6612_BIN1_PORT    MOTOR_BIN1_PORT
#define TB6612_BIN1_PIN     MOTOR_BIN1_PIN
#define TB6612_BIN2_PORT    MOTOR_BIN2_PORT
#define TB6612_BIN2_PIN     MOTOR_BIN2_PIN

// PWM配置 - 已按提供的配置修改
#define TB6612_PWMA_CHANNEL MOTOR_PWMA_CHAN
#define TB6612_PWMB_CHANNEL MOTOR_PWMB_CHAN
#define TB6612_PWM_MAX      MOTOR_PWM_MAX

// 函数声明
void TB6612_Init(void);
void TB6612_SetDirection(TB6612_MotorTypeDef motor, TB6612_DirectionTypeDef direction);
void TB6612_SetSpeed(TB6612_MotorTypeDef motor, uint16_t speed);
void TB6612_SmoothAccelerate(TB6612_MotorTypeDef motor, uint16_t target_speed,
                             uint16_t step, uint32_t delay_ms);
void TB6612_SmoothDecelerate(TB6612_MotorTypeDef motor, uint16_t target_speed,
                             uint16_t step, uint32_t delay_ms);
void TB6612_SoftStop(TB6612_MotorTypeDef motor, uint16_t step, uint32_t delay_ms);
void TB6612_HardStop(TB6612_MotorTypeDef motor);
void TB6612_Brake(TB6612_MotorTypeDef motor);
uint16_t TB6612_GetCurrentSpeed(TB6612_MotorTypeDef motor);
#endif //TWIGO_TB6612_H
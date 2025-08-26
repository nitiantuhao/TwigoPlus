//
// Created by Falling_jasmine on 2025/8/25.
//

#ifndef TWIGO_HC05_H
#define TWIGO_HC05_H
#include "stm32f1xx_hal.h"  // 根据实际MCU型号修改
#include <string.h>
#include <stdio.h>
extern UART_HandleTypeDef *hc05_huart;

// 修改HC05_Init函数声明
void HC05_Init(UART_HandleTypeDef *huart);

// HC-05配置参数结构体
typedef struct {
  char name[32];         // 蓝牙名称
  char pin[8];           // 配对密码
  uint32_t baud_rate;    // 波特率
  uint8_t role;          // 角色: 0-从机, 1-主机
} HC05_ConfigTypeDef;

// 蓝牙模块状态
typedef enum {
  HC05_OK = 0,
  HC05_ERROR,
  HC05_TIMEOUT,
  HC05_NOT_CONNECTED
} HC05_StatusTypeDef;

// 函数声明
void HC05_Init(UART_HandleTypeDef *huart);
HC05_StatusTypeDef HC05_SendData(uint8_t *data, uint16_t len);
HC05_StatusTypeDef HC05_ReceiveData(uint8_t *data, uint16_t max_len, uint32_t timeout);
HC05_StatusTypeDef HC05_SendATCommand(const char *cmd, char *response, uint16_t resp_len, uint32_t timeout);
HC05_StatusTypeDef HC05_GetConfig(HC05_ConfigTypeDef *config);
HC05_StatusTypeDef HC05_SetName(const char *name);
HC05_StatusTypeDef HC05_SetPin(const char *pin);
HC05_StatusTypeDef HC05_SetBaudRate(uint32_t baud_rate);
HC05_StatusTypeDef HC05_SetRole(uint8_t role);
HC05_StatusTypeDef HC05_Reset(void);
#endif //TWIGO_HC05_H
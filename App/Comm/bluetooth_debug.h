//
// Created by Falling_jasmine on 2025/8/25.
//

#ifndef TWIGO_BLUETOOTH_DEBUG_H
#define TWIGO_BLUETOOTH_DEBUG_H
#include "stm32f1xx_hal.h"
#include "hc05.h"

/**
 * @brief 初始化蓝牙调试功能
 * @param huart: 蓝牙模块连接的UART句柄
 */
void Bluetooth_Debug_Init(UART_HandleTypeDef *huart);

/**
 * @brief 蓝牙接收数据回调处理函数
 * @param huart: 发生中断的UART句柄
 * @note 需要在UART中断服务程序中调用
 */
void HC05_RxCallback(UART_HandleTypeDef *huart);

/**
 * @brief 通过蓝牙发送字符串通过蓝牙模块
 * @param str: 要发送的字符串
 */
void HC05_SendString(const char *str);

#endif //TWIGO_BLUETOOTH_DEBUG_H
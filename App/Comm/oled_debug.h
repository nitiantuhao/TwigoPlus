//
// Created by Falling_jasmine on 2025/8/25.
//

#ifndef TWIGO_OLED_DEBUG_H
#define TWIGO_OLED_DEBUG_H
#include "stm32f1xx_hal.h"

/**
 * @brief 初始化OLED调试功能
 */
void OLED_Debug_Init(void);

/**
 * @brief 更新调试信息并显示
 * @note 建议在主循环中调用
 */
void OLED_UpdateDebugInfo(void);
#endif //TWIGO_OLED_DEBUG_H
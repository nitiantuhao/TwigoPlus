#include "oled_debug.h"

#include <stdlib.h>

#include "oled.h"
#include "Balance/balance_control.h"
#include "Motor/tb6612.h"
#include "font.h"  // 假设包含默认字体定义

// 调试界面刷新间隔(ms)
#define DEBUG_REFRESH_INTERVAL 100

// 字体选择(根据实际字体库调整)
#define DEBUG_FONT & afont12x6 // 8x16 ASCII字体

/**
 * @brief 初始化OLED调试功能
 */
void OLED_Debug_Init(void) {
    OLED_Init();
    OLED_DisPlay_On();
    OLED_NewFrame();
    OLED_PrintASCIIString(0, 0, "Debug Mode", DEBUG_FONT, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    HAL_Delay(1000);  // 显示初始化信息
}

/**
 * @brief 格式化浮点数为字符串
 * @param num 要格式化的浮点数
 * @param str 输出字符串缓冲区
 * @param decimals 保留小数位数
 */
static void float_to_str(float num, char *str, uint8_t decimals) {
    int integer_part = (int)num;
    float fractional_part = num - integer_part;
    
    // 处理负数
    if (num < 0) {
        *str++ = '-';
        integer_part = -integer_part;
        fractional_part = -fractional_part;
    }
    
    // 转换整数部分
    itoa(integer_part, str, 10);
    while (*str != '\0') str++;
    
    // 添加小数点和小数部分
    *str++ = '.';
    for (uint8_t i = 0; i < decimals; i++) {
        fractional_part *= 10;
        *str++ = (int)fractional_part + '0';
        fractional_part -= (int)fractional_part;
    }
    *str = '\0';
}

/**
 * @brief 更新调试信息并显示
 */
void OLED_UpdateDebugInfo(void) {
    static uint32_t last_update_time = 0;
    char buffer[32];
    
    // 控制刷新频率
    if (HAL_GetTick() - last_update_time < DEBUG_REFRESH_INTERVAL) {
        return;
    }
    last_update_time = HAL_GetTick();
    
    // 开始绘制新帧
    OLED_NewFrame();
    
    // 显示PID参数
    OLED_PrintASCIIString(0, 0, "PID Parameters:", DEBUG_FONT, OLED_COLOR_NORMAL);
    
    // 显示P参数
    float_to_str(balance_pid.kp, buffer, 2);
    OLED_PrintASCIIString(0, 16, "P:", DEBUG_FONT, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(24, 16, buffer, DEBUG_FONT, OLED_COLOR_NORMAL);
    
    // 显示I参数
    float_to_str(balance_pid.ki, buffer, 2);
    OLED_PrintASCIIString(0, 32, "I:", DEBUG_FONT, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(24, 32, buffer, DEBUG_FONT, OLED_COLOR_NORMAL);
    
    // 显示D参数
    float_to_str(balance_pid.kd, buffer, 2);
    OLED_PrintASCIIString(0, 48, "D:", DEBUG_FONT, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(24, 48, buffer, DEBUG_FONT, OLED_COLOR_NORMAL);
    
    // 显示电机A速度
    itoa(TB6612_GetCurrentSpeed(TB6612_MOTOR_A), buffer, 10);
    OLED_PrintASCIIString(64, 16, "A:", DEBUG_FONT, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(88, 16, buffer, DEBUG_FONT, OLED_COLOR_NORMAL);
    
    // 显示电机B速度
    itoa(TB6612_GetCurrentSpeed(TB6612_MOTOR_B), buffer, 10);
    OLED_PrintASCIIString(64, 32, "B:", DEBUG_FONT, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(88, 32, buffer, DEBUG_FONT, OLED_COLOR_NORMAL);
    
    // 显示当前角度
    float_to_str(current_pitch, buffer, 1);
    OLED_PrintASCIIString(64, 48, "Angle:", DEBUG_FONT, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(104, 48, buffer, DEBUG_FONT, OLED_COLOR_NORMAL);
    
    // 刷新显示
    OLED_ShowFrame();
}
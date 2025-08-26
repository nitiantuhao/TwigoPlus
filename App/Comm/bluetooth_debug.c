//
// Created by Falling_jasmine on 2025/8/25.
//
#include "Comm/hc05.h"
#include "Balance/balance_control.h"
#include "Motor/tb6612.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// 蓝牙接收缓冲区配置
#define RX_BUF_SIZE 128
static uint8_t rx_buf[RX_BUF_SIZE] = {0};
static uint16_t rx_len = 0;
static uint8_t rx_temp;  // 中断接收临时变量

// 指令类型枚举
typedef enum {
    CMD_UNKNOWN,
    CMD_GET_INFO,
    CMD_SET_P,
    CMD_SET_I,
    CMD_SET_D,
    CMD_SET_TARGET
} CmdType;

// 解析指令类型
static CmdType get_cmd_type(const char *cmd) {
    if (strcmp(cmd, "get") == 0) {
        return CMD_GET_INFO;
    } else if (strncmp(cmd, "P ", 2) == 0) {
        return CMD_SET_P;
    } else if (strncmp(cmd, "I ", 2) == 0) {
        return CMD_SET_I;
    } else if (strncmp(cmd, "D ", 2) == 0) {
        return CMD_SET_D;
    } else if (strncmp(cmd, "T ", 2) == 0) {
        return CMD_SET_TARGET;
    }
    return CMD_UNKNOWN;
}

// 发送字符串封装
void HC05_SendString(const char *str) {
    if (str == NULL) return;
    HC05_SendData((uint8_t *)str, strlen(str));
}

// 处理参数设置指令
static void handle_param_set(CmdType type, const char *param_str) {
    float value;
    char reply[64];

    // 尝试解析数值
    if (sscanf(param_str, "%f", &value) != 1) {
        HC05_SendString("参数格式错误，请输入数字\r\n");
        return;
    }

    // 根据指令类型设置参数
    switch (type) {
        case CMD_SET_P:
            balance_pid.kp = value;
            snprintf(reply, sizeof(reply), "已设置P=%.2f\r\n", value);
            break;
        case CMD_SET_I:
            balance_pid.ki = value;
            snprintf(reply, sizeof(reply), "已设置I=%.2f\r\n", value);
            break;
        case CMD_SET_D:
            balance_pid.kd = value;
            snprintf(reply, sizeof(reply), "已设置D=%.2f\r\n", value);
            break;
        case CMD_SET_TARGET:
            balance_pid.target = value;
            snprintf(reply, sizeof(reply), "已设置目标角度=%.1f\r\n", value);
            break;
        default:
            return;
    }
    HC05_SendString(reply);
}

// 处理信息查询指令
static void handle_get_info(void) {
    char reply[128];
    snprintf(reply, sizeof(reply),
            "当前状态:\r\n"
            "PID参数: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n"
            "角度: 目标=%.1f°, 当前=%.1f°\r\n"
            "电机速度: A=%d, B=%d\r\n",
            balance_pid.kp, balance_pid.ki, balance_pid.kd,
            balance_pid.target, current_pitch,
            TB6612_GetCurrentSpeed(TB6612_MOTOR_A),
            TB6612_GetCurrentSpeed(TB6612_MOTOR_B));
    HC05_SendString(reply);
}

// 解析蓝牙指令主函数
static void ParseBluetoothCommand(uint8_t *rx_buf, uint16_t rx_len) {
    // 移除末尾换行符
    int effective_len = rx_len;
    while (effective_len > 0) {
        uint8_t last_char = rx_buf[effective_len - 1];
        if (last_char == '\r' || last_char == '\n') {
            effective_len--;
        } else {
            break;
        }
    }

    // 添加字符串结束符
    if (effective_len < RX_BUF_SIZE) {
        rx_buf[effective_len] = '\0';
    } else {
        rx_buf[RX_BUF_SIZE - 1] = '\0';
    }

    // 调试信息回显
    char dbg[64];
    snprintf(dbg, sizeof(dbg), "收到指令: %s\r\n", rx_buf);
    HC05_SendString(dbg);

    // 解析并执行指令
    CmdType cmd = get_cmd_type((char*)rx_buf);
    switch (cmd) {
        case CMD_GET_INFO:
            handle_get_info();
            break;
        case CMD_SET_P:
        case CMD_SET_I:
        case CMD_SET_D:
        case CMD_SET_TARGET:
            handle_param_set(cmd, (char*)rx_buf + 2);  // 跳过指令前缀
            break;
        default:
            HC05_SendString("未知指令!\r\n支持的指令:\r\n"
                           "  get - 查看当前状态\r\n"
                           "  P <值> - 设置比例系数\r\n"
                           "  I <值> - 设置积分系数\r\n"
                           "  D <值> - 设置微分系数\r\n"
                           "  T <值> - 设置目标角度\r\n");
            break;
    }
}

// 蓝牙数据接收回调函数
void HC05_RxCallback(UART_HandleTypeDef *huart) {
    // 过滤非蓝牙UART中断
    if (huart != hc05_huart) {
        return;
    }

    // 处理接收字符
    if (rx_temp == '\r' || rx_temp == '\n') {
        // 收到换行符且缓冲区有数据时解析
        if (rx_len > 0) {
            ParseBluetoothCommand(rx_buf, rx_len);
            // 重置缓冲区
            rx_len = 0;
            memset(rx_buf, 0, RX_BUF_SIZE);
        }
    } else {
        // 普通字符存入缓冲区(预留结束符空间)
        if (rx_len < RX_BUF_SIZE - 1) {
            rx_buf[rx_len++] = rx_temp;
        } else {
            // 缓冲区满时强制解析
            ParseBluetoothCommand(rx_buf, rx_len);
            rx_len = 0;
            memset(rx_buf, 0, RX_BUF_SIZE);
        }
    }

    // 重新开启接收中断
    HAL_UART_Receive_IT(huart, &rx_temp, 1);
}

// 初始化蓝牙调试功能
void Bluetooth_Debug_Init(UART_HandleTypeDef *huart) {
    HC05_Init(huart);
    // 开启UART接收中断
    HAL_UART_Receive_IT(huart, &rx_temp, 1);
    // 发送初始化提示
    HC05_SendString("蓝牙调试功能已启动\r\n");
    HC05_SendString("请发送以下指令:\r\n"
                   "  get - 查看当前状态\r\n"
                   "  P <值> - 设置PID比例系数\r\n"
                   "  I <值> - 设置PID积分系数\r\n"
                   "  D <值> - 设置PID微分系数\r\n"
                   "  T <值> - 设置目标平衡角度\r\n");
}
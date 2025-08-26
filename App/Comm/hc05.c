#include "hc05.h"

// 外部UART句柄引用
UART_HandleTypeDef *hc05_huart;

// 波特率对应AT指令值映射表
static const struct {
    uint32_t baud;
    char cmd[4];
} baud_rate_map[] = {
    {2400,    "0"},
    {4800,    "1"},
    {9600,    "2"},   // 默认
    {19200,   "3"},
    {38400,   "4"},
    {57600,   "5"},
    {115200,  "6"},
    {230400,  "7"},
    {460800,  "8"},
    {921600,  "9"},
};
/**
 * @brief  向HC05发送数据
 * @param  huart: UART句柄
 * @param  data: 要发送的数据
 * @param  len: 数据长度
 * @retval 无
 */

/**
 * @brief  初始化HC-05蓝牙模块
 * @param  huart: UART句柄 (已由CubeMX初始化)
 * @retval 无
 */
void HC05_Init(UART_HandleTypeDef *huart) {
  hc05_huart = huart;
  __HAL_UART_FLUSH_DRREGISTER(huart);
}


/**
 * @brief  发送数据通过蓝牙模块
 * @param  data: 要发送的数据
 * @param  len: 数据长度
 * @retval 状态: HC05_OK成功, HC05_ERROR失败
 */
HC05_StatusTypeDef HC05_SendData(uint8_t *data, uint16_t len) {
  if (HAL_UART_Transmit(hc05_huart, data, len, 100) == HAL_OK) {
    return HC05_OK;
  }
  return HC05_ERROR;
}

/**
 * @brief  从蓝牙模块接收数据
 * @param  data: 接收缓冲区
 * @param  max_len: 最大接收长度
 * @param  timeout: 超时时间(ms)
 * @retval 状态: HC05_OK成功, HC05_TIMEOUT超时
 */
HC05_StatusTypeDef HC05_ReceiveData(uint8_t *data, uint16_t max_len, uint32_t timeout) {
  if (HAL_UART_Receive(hc05_huart, data, max_len, timeout) == HAL_OK) {
    return HC05_OK;
  }
  return HC05_TIMEOUT;
}


/**
 * @brief  发送AT指令并获取响应
 * @param  cmd: AT指令
 * @param  response: 响应缓冲区
 * @param  resp_len: 响应缓冲区长度
 * @param  timeout: 超时时间(ms)
 * @retval 状态: HC05_OK成功, 其他失败
 */
HC05_StatusTypeDef HC05_SendATCommand(const char *cmd, char *response, uint16_t resp_len, uint32_t timeout) {
    if (resp_len == 0 || cmd == NULL) {
        return HC05_ERROR;
    }

    // 清空响应缓冲区
    memset(response, 0, resp_len);

    // 发送AT指令
    if (HAL_UART_Transmit(hc05_huart, (uint8_t*)cmd, strlen(cmd), 1000) != HAL_OK) {
        return HC05_ERROR;
    }

    // 等待响应
    if (HAL_UART_Receive(hc05_huart, (uint8_t*)response, resp_len-1, timeout) != HAL_OK) {
        return HC05_TIMEOUT;
    }

    // 检查是否收到OK响应
    if (strstr(response, "OK") == NULL && strstr(response, "ok") == NULL) {
        return HC05_ERROR;
    }

    return HC05_OK;
}

/**
 * @brief  获取当前配置
 * @param  config: 配置结构体指针
 * @retval 状态
 */
HC05_StatusTypeDef HC05_GetConfig(HC05_ConfigTypeDef *config) {
    char response[128];

    // 获取名称
    if (HC05_SendATCommand("AT+NAME?\r\n", response, sizeof(response), 1000) == HC05_OK) {
        sscanf(response, "+NAME:%s", config->name);
    } else {
        return HC05_ERROR;
    }

    // 获取密码
    if (HC05_SendATCommand("AT+PIN?\r\n", response, sizeof(response), 1000) == HC05_OK) {
        sscanf(response, "+PIN:%s", config->pin);
    } else {
        return HC05_ERROR;
    }

    // 获取波特率
    if (HC05_SendATCommand("AT+BAUD?\r\n", response, sizeof(response), 1000) == HC05_OK) {
        int baud_idx;
        sscanf(response, "+BAUD:%d", &baud_idx);
        if (baud_idx >= 0 && baud_idx < sizeof(baud_rate_map)/sizeof(baud_rate_map[0])) {
            config->baud_rate = baud_rate_map[baud_idx].baud;
        }
    } else {
        return HC05_ERROR;
    }

    // 获取角色
    if (HC05_SendATCommand("AT+ROLE?\r\n", response, sizeof(response), 1000) == HC05_OK) {
        sscanf(response, "+ROLE:%hhu", &config->role);
    } else {
        return HC05_ERROR;
    }

    return HC05_OK;
}

/**
 * @brief  设置蓝牙名称
 * @param  name: 设备名称
 * @retval 状态
 */
HC05_StatusTypeDef HC05_SetName(const char *name) {
    char cmd[64];
    char response[32];

    snprintf(cmd, sizeof(cmd), "AT+NAME%s\r\n", name);
    return HC05_SendATCommand(cmd, response, sizeof(response), 1000);
}

/**
 * @brief  设置配对密码
 * @param  pin: 4位数字密码
 * @retval 状态
 */
HC05_StatusTypeDef HC05_SetPin(const char *pin) {
    char cmd[32];
    char response[32];

    snprintf(cmd, sizeof(cmd), "AT+PIN%s\r\n", pin);
    return HC05_SendATCommand(cmd, response, sizeof(response), 1000);
}

/**
 * @brief  设置波特率
 * @param  baud_rate: 波特率值
 * @retval 状态
 */
HC05_StatusTypeDef HC05_SetBaudRate(uint32_t baud_rate) {
    char cmd[32];
    char response[32];
    uint8_t i;

    // 查找对应的波特率指令
    for (i = 0; i < sizeof(baud_rate_map)/sizeof(baud_rate_map[0]); i++) {
        if (baud_rate_map[i].baud == baud_rate) {
            snprintf(cmd, sizeof(cmd), "AT+BAUD%s\r\n", baud_rate_map[i].cmd);
            return HC05_SendATCommand(cmd, response, sizeof(response), 1000);
        }
    }

    return HC05_ERROR;
}

/**
 * @brief  设置角色(主机/从机)
 * @param  role: 0-从机, 1-主机
 * @retval 状态
 */
HC05_StatusTypeDef HC05_SetRole(uint8_t role) {
    char cmd[32];
    char response[32];

    if (role > 1) role = 0; // 角色只能是0或1

    snprintf(cmd, sizeof(cmd), "AT+ROLE%d\r\n", role);
    return HC05_SendATCommand(cmd, response, sizeof(response), 1000);
}

/**
 * @brief  重置蓝牙模块
 * @retval 状态
 */
HC05_StatusTypeDef HC05_Reset(void) {
    char response[32];
    return HC05_SendATCommand("AT+RESET\r\n", response, sizeof(response), 2000);
}

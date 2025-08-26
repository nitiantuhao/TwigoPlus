#include "tb6612.h"

// 外部引用定时器句柄
#define TB6612_PWM_TIM MOTOR_PWM_TIM

// 存储当前速度
uint16_t current_speed_a = 0;
uint16_t current_speed_b = 0;

/**
 * @brief  初始化TB6612电机驱动
 * @retval 无
 */
void TB6612_Init(void) {
    // 初始化时停止所有电机
    TB6612_SetDirection(TB6612_MOTOR_A, TB6612_STOP);
    TB6612_SetDirection(TB6612_MOTOR_B, TB6612_STOP);

    // 启动PWM (定时器已由CubeMX初始化)
    HAL_TIM_PWM_Start(TB6612_PWM_TIM, TB6612_PWMA_CHANNEL);
    HAL_TIM_PWM_Start(TB6612_PWM_TIM, TB6612_PWMB_CHANNEL);

    // 设置初始速度为0
    TB6612_SetSpeed(TB6612_MOTOR_A, 0);
    TB6612_SetSpeed(TB6612_MOTOR_B, 0);
}

/**
 * @brief  设置电机方向
 * @param  motor: 电机编号
 * @param  direction: 电机方向
 * @retval 无
 */
void TB6612_SetDirection(TB6612_MotorTypeDef motor, TB6612_DirectionTypeDef direction) {
    if (motor == TB6612_MOTOR_A) {
        switch(direction) {
            case TB6612_FORWARD:
                HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, GPIO_PIN_RESET);
                break;
            case TB6612_BACKWARD:
                HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, GPIO_PIN_SET);
                break;
            case TB6612_STOP:
                HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, GPIO_PIN_RESET);
                break;
        }
    } else if (motor == TB6612_MOTOR_B) {
        switch(direction) {
            case TB6612_FORWARD:
                HAL_GPIO_WritePin(TB6612_BIN1_PORT, TB6612_BIN1_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(TB6612_BIN2_PORT, TB6612_BIN2_PIN, GPIO_PIN_RESET);
                break;
            case TB6612_BACKWARD:
                HAL_GPIO_WritePin(TB6612_BIN1_PORT, TB6612_BIN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TB6612_BIN2_PORT, TB6612_BIN2_PIN, GPIO_PIN_SET);
                break;
            case TB6612_STOP:
                HAL_GPIO_WritePin(TB6612_BIN1_PORT, TB6612_BIN1_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TB6612_BIN2_PORT, TB6612_BIN2_PIN, GPIO_PIN_RESET);
                break;
        }
    }
}

/**
 * @brief  设置电机速度
 * @param  motor: 电机编号
 * @param  speed: 速度值 (0-TB6612_PWM_MAX)
 * @retval 无
 */
void TB6612_SetSpeed(TB6612_MotorTypeDef motor, uint16_t speed) {
    // 限制速度范围
    if (speed > TB6612_PWM_MAX) {
        speed = TB6612_PWM_MAX;
    }

    // 设置PWM比较值
    if (motor == TB6612_MOTOR_A) {
        __HAL_TIM_SET_COMPARE(TB6612_PWM_TIM, TB6612_PWMA_CHANNEL, speed);
        current_speed_a = speed;
    } else if (motor == TB6612_MOTOR_B) {
        __HAL_TIM_SET_COMPARE(TB6612_PWM_TIM, TB6612_PWMB_CHANNEL, speed);
        current_speed_b = speed;
    }
}

/**
 * @brief  平滑加速
 * @param  motor: 电机编号
 * @param  target_speed: 目标速度
 * @param  step: 每次加速的步长
 * @param  delay_ms: 每步延迟时间(ms)
 * @retval 无
 */
void TB6612_SmoothAccelerate(TB6612_MotorTypeDef motor, uint16_t target_speed,
                             uint16_t step, uint32_t delay_ms) {
    uint16_t current_speed = TB6612_GetCurrentSpeed(motor);

    // 限制目标速度
    if (target_speed > TB6612_PWM_MAX) {
        target_speed = TB6612_PWM_MAX;
    }

    // 逐步加速
    while (current_speed < target_speed) {
        current_speed += step;
        if (current_speed > target_speed) {
            current_speed = target_speed;
        }
        TB6612_SetSpeed(motor, current_speed);
        HAL_Delay(delay_ms);
    }
}

/**
 * @brief  平滑减速
 * @param  motor: 电机编号
 * @param  target_speed: 目标速度
 * @param  step: 每次减速的步长
 * @param  delay_ms: 每步延迟时间(ms)
 * @retval 无
 */
void TB6612_SmoothDecelerate(TB6612_MotorTypeDef motor, uint16_t target_speed,
                             uint16_t step, uint32_t delay_ms) {
    uint16_t current_speed = TB6612_GetCurrentSpeed(motor);

    // 限制目标速度不能小于0
    if (target_speed > TB6612_PWM_MAX) {
        target_speed = 0;
    }

    // 逐步减速
    while (current_speed > target_speed) {
        current_speed -= step;
        if (current_speed < target_speed) {
            current_speed = target_speed;
        }
        TB6612_SetSpeed(motor, current_speed);
        HAL_Delay(delay_ms);
    }
}

/**
 * @brief  软停止(减速至停止)
 * @param  motor: 电机编号
 * @param  step: 每次减速的步长
 * @param  delay_ms: 每步延迟时间(ms)
 * @retval 无
 */
void TB6612_SoftStop(TB6612_MotorTypeDef motor, uint16_t step, uint32_t delay_ms) {
    TB6612_SmoothDecelerate(motor, 0, step, delay_ms);
    TB6612_SetDirection(motor, TB6612_STOP);
}

/**
 * @brief  硬停止(立即停止)
 * @param  motor: 电机编号
 * @retval 无
 */
void TB6612_HardStop(TB6612_MotorTypeDef motor) {
    TB6612_SetSpeed(motor, 0);
    TB6612_SetDirection(motor, TB6612_STOP);
}

/**
 * @brief  电机制动(短路制动)
 * @param  motor: 电机编号
 * @retval 无
 */
void TB6612_Brake(TB6612_MotorTypeDef motor) {
    if (motor == TB6612_MOTOR_A) {
        HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, GPIO_PIN_SET);
    } else if (motor == TB6612_MOTOR_B) {
        HAL_GPIO_WritePin(TB6612_BIN1_PORT, TB6612_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TB6612_BIN2_PORT, TB6612_BIN2_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief  获取当前电机速度
 * @param  motor: 电机编号
 * @retval 当前速度值
 */
uint16_t TB6612_GetCurrentSpeed(TB6612_MotorTypeDef motor) {
    if (motor == TB6612_MOTOR_A) {
        return current_speed_a;
    } else {
        return current_speed_b;
    }
}

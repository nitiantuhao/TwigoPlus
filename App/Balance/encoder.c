#include "encoder.h"
#include "tim.h"

// 编码器计数变量
static int32_t encoder_count[ENCODER_NUM] = {0};

// 初始化编码器
void Encoder_Init(void) {
    // 启动编码器定时器（假设左编码器用TIM2，右编码器用TIM3）
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 左编码器
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  // 右编码器

    // 清零计数
    Encoder_Clear_Count(ENCODER_LEFT);
    Encoder_Clear_Count(ENCODER_RIGHT);
}

// 获取编码器计数
int32_t Encoder_Get_Count(Encoder_TypeDef encoder) {
    if (encoder >= ENCODER_NUM) return 0;

    switch (encoder) {
        case ENCODER_LEFT:
            encoder_count[encoder] = (int16_t)TIM2->CNT;
            break;
        case ENCODER_RIGHT:
            encoder_count[encoder] = (int16_t)TIM3->CNT;
            break;
        default:
            break;
    }

    return encoder_count[encoder];
}

// 清零编码器计数
void Encoder_Clear_Count(Encoder_TypeDef encoder) {
    if (encoder >= ENCODER_NUM) return;

    switch (encoder) {
        case ENCODER_LEFT:
            TIM2->CNT = 0;
            break;
        case ENCODER_RIGHT:
            TIM3->CNT = 0;
            break;
        default:
            break;
    }
}

// 编码器定时器中断回调函数（如果需要）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        // 左编码器溢出处理（如果需要）
    } else if (htim->Instance == TIM3) {
        // 右编码器溢出处理（如果需要）
    }
}

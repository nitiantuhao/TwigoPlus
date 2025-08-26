#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"

// 编码器定义
typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_NUM
} Encoder_TypeDef;

// 函数声明
void Encoder_Init(void);
int32_t Encoder_Get_Count(Encoder_TypeDef encoder);
void Encoder_Clear_Count(Encoder_TypeDef encoder);

#endif // ENCODER_H

//
// Created by Falling_jasmine on 2025/8/26.
//

#ifndef TWIGO_ENCODER_H
#define TWIGO_ENCODER_H
#include "stm32f1xx_hal.h"

typedef enum {
  ENCODER_LEFT = 0,
  ENCODER_RIGHT,
  ENCODER_NUM
} Encoder_TypeDef;

void Encoder_Init(void);
int32_t Encoder_Get_Count(Encoder_TypeDef encoder);
void Encoder_Clear_Count(Encoder_TypeDef encoder);
#endif //TWIGO_ENCODER_H
#include "encoder.h"
#include "tim.h"

static int32_t encoder_count[ENCODER_NUM] = {0};

void Encoder_Init(void) {
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);   // 左轮 TIM2
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   // 右轮 TIM4
  Encoder_Clear_Count(ENCODER_LEFT);
  Encoder_Clear_Count(ENCODER_RIGHT);
}

int32_t Encoder_Get_Count(Encoder_TypeDef encoder) {
  if (encoder >= ENCODER_NUM) return 0;
  switch (encoder) {
    case ENCODER_LEFT:
      encoder_count[encoder] = (int16_t)TIM2->CNT;
      break;
    case ENCODER_RIGHT:
      encoder_count[encoder] = (int16_t)TIM4->CNT;   // ← 改这里
      break;
    default:
      break;
  }
  return encoder_count[encoder];
}

void Encoder_Clear_Count(Encoder_TypeDef encoder) {
  if (encoder >= ENCODER_NUM) return;
  switch (encoder) {
    case ENCODER_LEFT:
      TIM2->CNT = 0;
      break;
    case ENCODER_RIGHT:
      TIM4->CNT = 0;   // ← 改这里
      break;
    default:
      break;
  }
}
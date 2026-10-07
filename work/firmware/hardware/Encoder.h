#ifndef ENCODER_H
#define ENCODER_H
#include "stm32f10x.h"
void Encoder_Init(void);
/* 获取并清零一个采样周期的脉冲；返回原始方向，符号在 Track.c 统一。 */
int16_t Encoder_Get(int id);
void Encoder_GetPair(int16_t *left, int16_t *right);
void Encoder_Clear(int id);
#endif

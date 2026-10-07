#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H
#include "stm32f10x.h"
void Gray_Init(void);
/* 方向映射完成后，位 0 对应车身左端。 */
uint8_t Gray_Read(void);
#endif

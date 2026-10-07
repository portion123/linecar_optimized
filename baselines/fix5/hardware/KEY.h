#ifndef KEY_H
#define KEY_H
#include "stm32f10x.h"
#define KEY0_PRES 1U
#define KEY1_PRES 2U
#define KEY2_PRES 3U
#define KEY5_PRES 5U
void KEY_Init(void);
uint8_t KEY_Scan(uint8_t mode);
#endif

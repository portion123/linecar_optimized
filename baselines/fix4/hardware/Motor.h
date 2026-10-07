#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"


#define IN1 PBout(4)
#define IN2 PBout(5)
#define IN3 PBout(6)
#define IN4 PBout(7)

void Motor_Init(void);
void Motor_SetLeftspeed(int8_t Speed);
void Motor_SetRightspeed(int8_t Speed);

#endif

#ifndef JIAODU_H
#define JIAODU_H

#include <stdint.h>


void Gyro_Turn_To_Angle(float target_angle);          // ?????????
void Gyro_Go_Straight_With_Angle(float target_angle, int16_t base_speed);
void Gyro_Go_Straight_With_Angle1(float target_angle, int16_t base_speed);
void line(void);

#endif

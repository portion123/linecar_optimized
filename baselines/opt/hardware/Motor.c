/* 电机底层：传入的是带方向的 PWM 百分比，正数前进，负数后退，不是 RPM。 */
#include "stm32f10x.h"
#include "pwm.h"
#include "Motor.h"
#include "CarConfig.h"
void Motor_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    /* PB4 默认属于 JTAG。释放 JTAG 引脚，保留 PA13/PA14 的 SWD 下载功能。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    PWM_Init(); Motor_SetLeftspeed(0); Motor_SetRightspeed(0);
    /* STBY 沿用原来的硬件使能接法，没有擅自占用新 GPIO。 */
}
void Motor_SetLeftspeed(int8_t speed)
{
    int16_t value = (int16_t)speed * LEFT_MOTOR_SIGN;
    if (value > 100) value = 100;
    if (value < -100) value = -100;
    /* 先关 PWM 再切方向，最后恢复占空比。0 表示停止驱动。 */
    PWM_SetCompare3(0);
    if (value == 0) { IN1 = 0; IN2 = 0; }
    else if (value > 0) { IN1 = 1; IN2 = 0; }
    else { IN1 = 0; IN2 = 1; }
    PWM_SetCompare3((uint16_t)(value < 0 ? -value : value));
}
void Motor_SetRightspeed(int8_t speed)
{
    int16_t value = (int16_t)speed * RIGHT_MOTOR_SIGN;
    if (value > 100) value = 100;
    if (value < -100) value = -100;
    PWM_SetCompare4(0);
    /* 右电机底层方向关系延用原工程，不直接照搬左侧的 IN 组合。 */
    if (value == 0) { IN3 = 0; IN4 = 0; }
    else if (value > 0) { IN3 = 0; IN4 = 1; }
    else { IN3 = 1; IN4 = 0; }
    PWM_SetCompare4((uint16_t)(value < 0 ? -value : value));
}

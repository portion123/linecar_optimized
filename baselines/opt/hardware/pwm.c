/* TIM2 的 CH3/CH4 分别从 PA2/PA3 输出两轮 PWM。 */
#include "stm32f10x.h"
#include "pwm.h"
void PWM_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef channel;
    RCC_ClocksTypeDef clocks;
    uint32_t timer_clock;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_AF_PP; /* 引脚交给定时器输出，不能用普通推挽替代 */
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    TIM_InternalClockConfig(TIM2);
    RCC_GetClocksFreq(&clocks);
    timer_clock = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_clock *= 2U;
    /* 100 个计数为一个 PWM 周期。72 MHz 下 PSC=719，ARR=99，频率 1 kHz。
     * 根据实际总线时钟计算 PSC，避免改时钟后占空比频率跟着出错。 */
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Period = 100U - 1U;
    timer.TIM_Prescaler = (uint16_t)(timer_clock / 100000U - 1U);
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_OCStructInit(&channel);
    channel.TIM_OCMode = TIM_OCMode_PWM1;
    channel.TIM_OCPolarity = TIM_OCPolarity_High;
    channel.TIM_OutputState = TIM_OutputState_Enable;
    channel.TIM_Pulse = 0; /* 初始停止，避免上电就跑 */
    TIM_OC3Init(TIM2, &channel); TIM_OC4Init(TIM2, &channel);
    /* CCR 不开启预装载，使切方向前写入的 0 立即生效。 */
    TIM_Cmd(TIM2, ENABLE);
}
void PWM_SetCompare3(uint16_t compare) { TIM_SetCompare3(TIM2, compare > 100U ? 100U : compare); }
void PWM_SetCompare4(uint16_t compare) { TIM_SetCompare4(TIM2, compare > 100U ? 100U : compare); }

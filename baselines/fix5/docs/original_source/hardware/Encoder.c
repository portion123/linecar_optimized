/* 两路编码器使用 A/B 两相下降沿中断；保留原计数倍率。
 * 只有下降沿发生且另一相为低才计一次，不能按四倍频去填每圈计数。
 */
#include "stm32f10x.h"
#include "Encoder.h"
volatile int16_t Encoder_Count[2] = {0, 0};
void Encoder_Init(void)
{
    GPIO_InitTypeDef gpio;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef irq;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio);
    /* 左编码器 PB0/PB1，右编码器 PB14/PB15，沿用原接线。 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);
    EXTI_StructInit(&exti);
    exti.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line14 | EXTI_Line15;
    exti.EXTI_LineCmd = ENABLE; exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_ClearITPendingBit(exti.EXTI_Line); EXTI_Init(&exti);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    irq.NVIC_IRQChannel = EXTI0_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1; /* 高于 1 ms 计时中断 */
    irq.NVIC_IRQChannelSubPriority = 2; irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    irq.NVIC_IRQChannel = EXTI1_IRQn; irq.NVIC_IRQChannelSubPriority = 3; NVIC_Init(&irq);
    irq.NVIC_IRQChannel = EXTI15_10_IRQn; irq.NVIC_IRQChannelSubPriority = 1; NVIC_Init(&irq);
}
int16_t Encoder_Get(int id)
{
    int16_t value; uint32_t saved;
    if (id < 1 || id > 2) return 0;
    /* 暂时关中断，读完立即清零，最后恢复原中断状态，防止读/清之间丢脉冲。 */
    saved = __get_PRIMASK(); __disable_irq();
    value = Encoder_Count[id - 1]; Encoder_Count[id - 1] = 0;
    __set_PRIMASK(saved); return value;
}
void Encoder_GetPair(int16_t *left, int16_t *right)
{
    uint32_t saved;
    if (!left || !right) return;
    /* 两轮在同一临界区同时读清，减少左右采样时刻不一致。 */
    saved = __get_PRIMASK(); __disable_irq();
    *left = Encoder_Count[0]; *right = Encoder_Count[1];
    Encoder_Count[0] = Encoder_Count[1] = 0; __set_PRIMASK(saved);
}
void Encoder_Clear(int id) { (void)Encoder_Get(id); }
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0 &&
            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0) --Encoder_Count[0];
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0 &&
            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) ++Encoder_Count[0];
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0 &&
            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 0) --Encoder_Count[1];
        EXTI_ClearITPendingBit(EXTI_Line14);
    }
    if (EXTI_GetITStatus(EXTI_Line15) == SET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 0 &&
            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0) ++Encoder_Count[1];
        EXTI_ClearITPendingBit(EXTI_Line15);
    }
}

/* 感为八路并行数字输入，不操作模块 IIC/SDA/SCL。
 * 沿用原工程 PULL 开漏方式与 3.3V 上拉。PF6/PF7 不是 5V 容忍脚，
 * 模块的高电平不能直接以 5V 推挽送入这两个引脚，需核对硬件电平。
 */
#include "GraySensor.h"
#include "CarConfig.h"
void Gray_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = 0x00FFU; /* PF0～PF7 对应八路输出 */
    gpio.GPIO_Mode = GPIO_Mode_IPU; /* 输入上拉，接收开漏信号 */
    GPIO_Init(GPIOF, &gpio);
}
uint8_t Gray_Read(void)
{
    uint8_t mask = (uint8_t)(GPIO_ReadInputData(GPIOF) & 0x00FFU);
#if TRACK_BLACK_IS_LOW
    mask = (uint8_t)~mask; /* 把黑色低电平转换成软件里的 1 */
#endif
#if TRACK_SENSOR_REVERSED
    /* 需要反转探头顺序时，分三次交换相邻位、位对和半字节。 */
    mask = (uint8_t)(((mask & 0x55U) << 1) | ((mask >> 1) & 0x55U));
    mask = (uint8_t)(((mask & 0x33U) << 2) | ((mask >> 2) & 0x33U));
    mask = (uint8_t)((mask << 4) | (mask >> 4));
#endif
    return mask; /* 位 0 是物理左侧，1 为黑，0 为白 */
}

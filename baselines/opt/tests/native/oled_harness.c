/* 单独执行真实OLED显存/分片发送，GPIO替身只计数；不虚构STM32时延。 */
#include "stm32f10x.h"
#include "../../hardware/OLED.c"
static unsigned writes;
void GPIO_WriteBit(GPIO_TypeDef *port,uint16_t pin,BitAction bit)
{ (void)port; (void)pin; (void)bit; ++writes; }
void GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *gpio) { (void)port; (void)gpio; }
void RCC_APB2PeriphClockCmd(uint32_t port,FunctionalState on) { (void)port; (void)on; }
void RealOLED_Reset(void) { OLED_Clear(); writes=0; }
unsigned RealOLED_Writes(void) { return writes; }
void RealOLED_ClearWrites(void) { writes=0; }
unsigned RealOLED_Dirty(void)
{
    unsigned page,bit,count=0;
    for(page=0;page<8;++page) for(bit=0;bit<16;++bit) count+=(oled_dirty[page]>>bit)&1U;
    return count;
}
unsigned RealOLED_Pixel(unsigned page,unsigned column)
{ return page<8 && column<128 ? oled_buffer[page][column] : 0; }

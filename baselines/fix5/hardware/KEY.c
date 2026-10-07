/* 非阻塞消抖：稳定 20 ms 才认可，按住只产生一次事件，松开后才能再按。 */
#include "stm32f10x.h"
#include "KEY.h"
#include "Track.h"
#include "CarConfig.h"

static uint8_t candidate, stable, armed;
static uint32_t changed_at;

void KEY_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOE, &gpio);
    candidate = stable = armed = 0;
    changed_at = 0;
}

uint8_t KEY_Scan(uint8_t mode)
{
    uint8_t raw = 0;
    uint32_t now = Track_Now();
    (void)mode; /* 参数保留兼容，始终每次按下只产生一次事件。 */
    /* 停车键优先；同时按下启动与停车时执行停车。 */
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == Bit_RESET) raw = KEY0_PRES;
    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == Bit_RESET) raw = KEY2_PRES;
    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == Bit_RESET &&
             GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == Bit_RESET) raw = KEY0_PRES;
    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == Bit_RESET) raw = KEY1_PRES;
    else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == Bit_RESET) raw = KEY5_PRES;
    if (raw != candidate)
    {
        candidate = raw;
        changed_at = now;
    }
    if ((uint32_t)(now - changed_at) < KEY_DEBOUNCE_MS) return 0;
    if (!candidate)
    {
        stable = 0;
        armed = 1; /* 上电时就按住按键，也必须先松开才能启动。 */
        return 0;
    }
    if (candidate != stable)
    {
        stable = candidate;
        if (candidate == KEY0_PRES || candidate == KEY2_PRES)
        {
            armed = 0;
            return candidate;
        }
        if (armed)
        {
            armed = 0;
            return candidate;
        }
    }
    return 0;
}

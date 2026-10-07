#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include "Delay.h"


void ParseAndPrintData(uint8_t *data, uint16_t length);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type);

volatile float global_angle = 0;
volatile uint8_t new_data_received = 0;
volatile float angular_velocity_y = 0.0;
volatile float angular_velocity_z = 0.0;

void Usart3_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)
{
    static uint8_t rx_buffer[11];
    static uint8_t rx_index = 0;
    static uint8_t state = 0;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART3);

        switch(state)
        {
            case 0:
                if(data == 0x55)
                {
                    rx_buffer[0] = data;
                    rx_index = 1;
                    state = 1;
                }
                break;

            case 1:
                rx_buffer[rx_index++] = data;
                if(data == 0x53)
                {
                    state = 2;
                }
                else
                {
                    state = 0;
                    rx_index = 0;
                }
                break;

            case 2:
                rx_buffer[rx_index++] = data;
                if(rx_index >= 11)
                {
                    ParseAndPrintData(rx_buffer, 11);
                    state = 0;
                    rx_index = 0;
                }
                break;
        }
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

void ParseAndPrintData(uint8_t *data, uint16_t length)
{
    if (length == 11)
    {
        uint8_t checksum = CalculateChecksum(data, length - 1, data[1]);
        if (checksum != data[length - 1])
        {
            return;
        }

        if (data[0] == 0x55 && data[1] == 0x53)
        {
            uint8_t yaw_l = data[6];
            uint8_t yaw_h = data[7];
            int16_t yaw = (int16_t)((yaw_h << 8) | yaw_l);
            float angle = ((float)yaw / 32768.0) * 180.0;
            global_angle = angle;
            new_data_received = 1;
        }
    }
}

uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

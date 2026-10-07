#ifndef __USART_H
#define __USART_H
 
#include <stdio.h>
void Usart3_Init(void);
void Usart3_SendByte(uint8_t Byte);
 
void Usart3_SendString(char *String);
void Usart3_SendArray(uint8_t *array, uint16_t length);
 
void Usart3_Printf(char *format, ...);
void ParseAndPrintData(uint8_t *data, uint16_t length);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type);

#endif

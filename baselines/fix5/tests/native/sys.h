#ifndef NATIVE_SYS_H
#define NATIVE_SYS_H
/* 仅用于电脑测试：把位带输出脚映射成数组。此目录不加入 Keil 工程。 */
#include "stm32f10x.h"
extern uint32_t native_pb_pins[16];
#define PBout(n) native_pb_pins[(n)]
#endif

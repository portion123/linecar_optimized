#ifndef NATIVE_SYS_H
#define NATIVE_SYS_H
/* Host-only: bit-band GPIO outputs mapped to an array. Never in the Keil project. */
#include "stm32f10x.h"
extern uint32_t native_pb_pins[16];
#define PBout(n) native_pb_pins[(n)]
#endif

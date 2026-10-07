#ifndef TRACK_H
#define TRACK_H
#include "stm32f10x.h"
void Track_Init(void);            /* 初始化探头、计时器和控制状态 */
void Track_Start(void);           /* 探头有黑线才进入循迹 */
void Track_StartForward(void);    /* 人工固定 PWM 直行检查 */
void Track_HandleKey(uint8_t key); /* 处理单次按下事件 */
void Track_Stop(void);            /* 立即撤去电机驱动，清空控制记忆 */
void Track_Task(void);            /* 主循环反复调用，内部按时间分配任务 */
uint8_t Track_IsRunning(void);    /* 返回当前是否属于运动阶段 */
uint32_t Track_Now(void);         /* 开机至今的毫秒计数 */
#endif

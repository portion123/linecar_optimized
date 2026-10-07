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
/* FIX4_SAFE：首个停车原因锁存，直到再次 KEY1/KEY5 启动才清除。 */
typedef enum {
    TRACK_STOP_NONE=0, TRACK_STOP_USER=1,
    TRACK_STOP_ENC_NO_PULSE_L=2, TRACK_STOP_ENC_NO_PULSE_R=3,
    TRACK_STOP_ENC_REVERSE_L=4, TRACK_STOP_ENC_REVERSE_R=5,
    TRACK_STOP_CONTROL_GAP=6,
    TRACK_STOP_APPROACH_TIME=7, TRACK_STOP_APPROACH_DIST=8, TRACK_STOP_APPROACH_YAW=9,
    TRACK_STOP_CORNER_TIME=10, TRACK_STOP_CORNER_SIDES=11,
    TRACK_STOP_SEARCH_TIME=12, TRACK_STOP_SEARCH_SIDES=13,
    TRACK_STOP_EDGE_ANGLE=14, TRACK_STOP_EDGE_TIME=15,
    TRACK_STOP_REC_TIME=16, TRACK_STOP_REC_ATTEMPTS=17,
    TRACK_STOP_REC_ANGLE=18, TRACK_STOP_REC_DISTANCE=19
} TrackStopReason;
uint8_t Track_GetStopReason(void); /* 返回 TrackStopReason */
#endif

#ifndef TRACK_H
#define TRACK_H
#include "stm32f10x.h"
/* 上层显式声明几何运动方式，零 RPM 只用于受控停轮，不作为低速死区。 */
typedef enum {
    TRACK_MOTION_FORWARD=0, /* 两轮连续向前，允许内轮目标 <10RPM */
    TRACK_MOTION_PIVOT=1,   /* 一轮明确停转，另一轮向前 */
    TRACK_MOTION_SPIN=2     /* 两轮反向原地旋转 */
} TrackMotionMode;
typedef enum {
    CAR_READY=0, CAR_TRACK=1, CAR_SEARCH=2, CAR_NO_LINE=4,
    CAR_STOP=8, CAR_ENCODER_FAULT=9, CAR_FORWARD=10,
    CAR_TURN_FAST=11, CAR_CORNER_CONFIRM=12, CAR_CORNER_APPROACH=14,
    CAR_EXIT=15, CAR_TURN_SLOW=16, CAR_ALIGN=17, CAR_CONTROL_FAULT=18, CAR_SPEED_TEST=19
} TrackState;
typedef enum {
    TRACK_STOP_NONE=0, TRACK_STOP_USER=1, TRACK_STOP_ENCODER=2,
    TRACK_STOP_PERIOD=3, TRACK_STOP_CONFIRM=4, TRACK_STOP_APPROACH=5,
    TRACK_STOP_TURN=6, TRACK_STOP_ALIGN=7, TRACK_STOP_EXIT=8,
    TRACK_STOP_SEARCH=9, TRACK_STOP_RECOVERY_TIME=10,
    TRACK_STOP_RECOVERY_ANGLE=11, TRACK_STOP_RECOVERY_DISTANCE=12,
    TRACK_STOP_RECOVERY_ATTEMPTS=13, TRACK_STOP_NO_DIRECTION=14,
    TRACK_STOP_MODE=15, TRACK_STOP_AMBIGUOUS=16, TRACK_STOP_FORWARD_TIME=17, TRACK_STOP_BENCH_TIME=18
} TrackStopReason;
/* 固定大小调试快照：RPM和角度放大10倍，距离mm。无阻塞串口输出。 */
typedef struct {
    uint32_t now_ms,recovery_age_ms,control_elapsed_ms,max_control_elapsed_ms;
    int16_t left_target10,right_target10,left_request10,right_request10;
    int16_t left_actual10,right_actual10,angle_deg10;
    uint16_t approach_mm,approach_goal_mm,recovery_angle_deg10,recovery_distance_mm;
    int8_t line_error10,turn_hint,corner_dir,left_pwm,right_pwm;
    uint8_t state,mode,sensors,confidence,recovery_active,recovery_attempts,stop_reason;
} TrackDebug;
void Track_GetDebug(TrackDebug *sample); /* 主循环中读取；有空再送入自选USART发送队列 */
void Track_StartSpeedTest(int16_t left_rpm,int16_t right_rpm); /* 架空单轮正/反闭环测试，限时限速 */
void Track_Init(void);            /* 初始化探头、计时器和控制状态 */
void Track_Start(void);           /* 探头有黑线才进入循迹 */
void Track_StartForward(void);    /* 人工固定 PWM 直行检查 */
void Track_HandleKey(uint8_t key); /* 处理单次按下事件 */
void Track_Stop(void);            /* 立即撤去电机驱动，清空控制记忆 */
void Track_Task(void);            /* 主循环反复调用，内部按时间分配任务 */
uint8_t Track_IsRunning(void);    /* 返回当前是否属于运动阶段 */
uint32_t Track_Now(void);         /* 开机至今的毫秒计数 */
#endif

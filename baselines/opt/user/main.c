/* 主程序只负责初始化和反复调用任务，所有循迹阶段由 Track.c 管理。 */
#include "stm32f10x.h"
#include "Motor.h"
#include "Encoder.h"
#include "KEY.h"
#include "Track.h"
#include "OLED.h"
#include "CarConfig.h"
int main(void)
{
    uint8_t key;
    /* 启动文件已经调用 SystemInit，这里只更新时钟变量。 */
    SystemCoreClockUpdate(); NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Motor_Init(); /* 第一步：方向脚和 PWM 初始化，电机先停止 */
    KEY_Init();   /* 第二步：配置按键输入及消抖状态 */
    Encoder_Init(); /* 第三步：配置左右编码器下降沿中断 */
#if TRACK_OLED_ENABLED
    OLED_Init(); /* 第四步：初始化屏幕，只显示状态，不参与判断路线 */
#endif
    Track_Init(); /* 第五步：初始化探头、控制状态、1 ms 计时器 */
#if TRACK_BENCH_TEST
    Track_StartSpeedTest(TRACK_BENCH_LEFT_RPM,TRACK_BENCH_RIGHT_RPM);
#endif
    while (1) {
        key = KEY_Scan(0); /* 扫描按键，不使用阻塞式延时 */
        Track_HandleKey(key); /* 有按键事件时启动或停车 */
        Track_Task(); /* 到 20 ms 才执行控制，空闲时分行刷新 OLED */
    }
}

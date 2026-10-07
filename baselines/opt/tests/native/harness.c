/* 在电脑上执行真实控制 C 源码；只替换硬件寄存器读写，不复制控制算法。
 * 这不是 ARM 固件仿真，也不证明实车动力学或 Keil 最终链接成功。
 */
#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
uint32_t native_pb_pins[16];
static uint16_t gray_raw=255, key_raw=65535, encoder_gpio=65535;
static uint32_t irq_mask, exti_pending;
static char screen[4][17];
static unsigned oled_invalid;
static unsigned oled_services;
static uint16_t compare3,compare4,pwm_prescaler,clock_prescaler;
static uint16_t gray_pin_mask;
static int gray_mode;
/* 保留实际 Encoder.c，仅把三条 ARM 中断屏蔽指令改为电脑状态变量。 */
#define __get_PRIMASK() irq_mask
#define __disable_irq() (irq_mask=1)
#define __set_PRIMASK(x) (irq_mask=(x))
#include "../../hardware/Encoder.c"
#include "../../hardware/GraySensor.c"
#include "../../hardware/KEY.c"
#include "../../hardware/pwm.c"
#include "../../hardware/Motor.c"
#include "../../system/PID.c"
#include "../../hardware/Track.c"

void RCC_APB2PeriphClockCmd(uint32_t p,FunctionalState s) { (void)p;(void)s; }
void RCC_APB1PeriphClockCmd(uint32_t p,FunctionalState s) { (void)p;(void)s; }
void RCC_GetClocksFreq(RCC_ClocksTypeDef *c)
{
    memset(c,0,sizeof(*c)); c->SYSCLK_Frequency=c->HCLK_Frequency=72000000;
    c->PCLK1_Frequency=36000000; c->PCLK2_Frequency=72000000;
}
void GPIO_StructInit(GPIO_InitTypeDef *g) { memset(g,0,sizeof(*g)); }
void GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g)
{ if(p==GPIOF) { gray_pin_mask=g->GPIO_Pin; gray_mode=g->GPIO_Mode; } }
void GPIO_ResetBits(GPIO_TypeDef *p,uint16_t b)
{ unsigned i; if(p==GPIOB) for(i=0;i<16;++i) if(b&(1U<<i)) native_pb_pins[i]=0; }
void GPIO_PinRemapConfig(uint32_t p,FunctionalState s) { (void)p;(void)s; }
void GPIO_EXTILineConfig(uint8_t p,uint8_t b) { (void)p;(void)b; }
uint16_t GPIO_ReadInputData(GPIO_TypeDef *p)
{ return p==GPIOF ? gray_raw : (p==GPIOE ? key_raw : encoder_gpio); }
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *p,uint16_t b) { return (GPIO_ReadInputData(p)&b)!=0; }
void NVIC_PriorityGroupConfig(uint32_t p) { (void)p; }
void NVIC_Init(NVIC_InitTypeDef *p) { (void)p; }
void EXTI_StructInit(EXTI_InitTypeDef *p) { memset(p,0,sizeof(*p)); }
void EXTI_Init(EXTI_InitTypeDef *p) { (void)p; }
void EXTI_ClearITPendingBit(uint32_t b) { exti_pending&=~b; }
ITStatus EXTI_GetITStatus(uint32_t b) { return (exti_pending&b) ? SET : RESET; }
void TIM_InternalClockConfig(TIM_TypeDef *p) { (void)p; }
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef *p) { memset(p,0,sizeof(*p)); }
void TIM_TimeBaseInit(TIM_TypeDef *p,TIM_TimeBaseInitTypeDef *t)
{ if(p==TIM2) pwm_prescaler=t->TIM_Prescaler; else clock_prescaler=t->TIM_Prescaler; }
void TIM_OCStructInit(TIM_OCInitTypeDef *p) { memset(p,0,sizeof(*p)); }
void TIM_OC3Init(TIM_TypeDef *p,TIM_OCInitTypeDef *t) { (void)p;compare3=t->TIM_Pulse; }
void TIM_OC4Init(TIM_TypeDef *p,TIM_OCInitTypeDef *t) { (void)p;compare4=t->TIM_Pulse; }
void TIM_SetCompare3(TIM_TypeDef *p,uint16_t x) { (void)p;compare3=x; }
void TIM_SetCompare4(TIM_TypeDef *p,uint16_t x) { (void)p;compare4=x; }
void TIM_ClearITPendingBit(TIM_TypeDef *p,uint16_t b) { (void)p;(void)b; }
ITStatus TIM_GetITStatus(TIM_TypeDef *p,uint16_t b) { (void)p;(void)b;return SET; }
void TIM_ITConfig(TIM_TypeDef *p,uint16_t b,FunctionalState s) { (void)p;(void)b;(void)s; }
void TIM_Cmd(TIM_TypeDef *p,FunctionalState s) { (void)p;(void)s; }

/* 把 OLED 输出写入 4×16 字符缓冲区，还检查越界，避免只把刷屏跳过。 */
void OLED_ShowChar(uint8_t line,uint8_t col,char c)
{ if(line<1 || line>4 || col<1 || col>16) ++oled_invalid; else screen[line-1][col-1]=c; }
void OLED_ShowString(uint8_t line,uint8_t col,char *s)
{ while(*s) OLED_ShowChar(line,col++,*s++); }
void OLED_ShowSignedNum(uint8_t line,uint8_t col,int32_t n,uint8_t len)
{
    char buffer[24]; uint32_t m=n<0 ? 0U-(uint32_t)n : (uint32_t)n;
    if(len>10) len=10;
    snprintf(buffer,sizeof(buffer),"%c%0*u",n<0 ? '-' : '+',len,m);
    OLED_ShowString(line,col,buffer);
}
void OLED_Service(uint8_t budget) { if(budget>8) ++oled_invalid; ++oled_services; }
void Native_Reset(void)
{
    memset(native_pb_pins,0,sizeof(native_pb_pins)); memset(screen,' ',sizeof(screen));
    for(unsigned i=0;i<4;++i) screen[i][16]=0;
    gray_raw=255; key_raw=encoder_gpio=65535; irq_mask=exti_pending=oled_invalid=oled_services=0;
    Encoder_Count[0]=Encoder_Count[1]=0; Motor_Init(); KEY_Init(); Encoder_Init(); Track_Init();
}
void Native_Sensors(unsigned mask) { gray_raw=(uint16_t)(~mask&255U); }
void Native_Start(unsigned mask) { Native_Reset(); Native_Sensors(mask); Track_Start(); }
void Native_Step(uint32_t now,int left,int right,unsigned mask)
{
#if DRIVE_PAIRS_SWAPPED
    Encoder_Count[0]=(int16_t)(LEFT_ENCODER_SIGN*right); Encoder_Count[1]=(int16_t)(RIGHT_ENCODER_SIGN*left);
#else
    Encoder_Count[0]=(int16_t)(LEFT_ENCODER_SIGN*left); Encoder_Count[1]=(int16_t)(RIGHT_ENCODER_SIGN*right);
#endif
    Native_Sensors(mask); milliseconds=now; Track_Task();
}
int Native_State(void) { return state; }
int Native_LeftPWM(void) { return left_pwm; }
int Native_RightPWM(void) { return right_pwm; }
float Native_LeftTarget(void) { return left_target; }
float Native_RightTarget(void) { return right_target; }
float Native_LeftCommand(void) { return left_command; }
float Native_RightCommand(void) { return right_command; }
float Native_Error(void) { return last_error; }
float Native_Approach(void) { return approach_mm; }
float Native_Goal(void) { return approach_goal_mm; }
float Native_Angle(void) { return turn_angle; }
float Native_LeftRPM(void) { return left_rpm; }
float Native_RightRPM(void) { return right_rpm; }
float Native_FrontOffset(void) { return SENSOR_FRONT_OFFSET_MM; }
float Native_AxleTrack(void) { return AXLE_TRACK_MM; }
int Native_Dir(void) { return corner_dir; }
int Native_Sweep(void) { return sweep_number; }
int Native_WideActive(void) { return wide_active; }
int Native_LineWidth(void) { return normal_line_width; }
int Native_WideMS(void) { return (int)wide_ms; }
int Native_LostMS(void) { return (int)lost_ms; }
int Native_OLEDInvalid(void) { return (int)oled_invalid; }
int Native_Pin(unsigned i) { return (int)native_pb_pins[i]; }
int Native_Compare(unsigned i) { return i==0 ? compare3 : compare4; }
int Native_Prescaler(unsigned i) { return i==0 ? pwm_prescaler : clock_prescaler; }
int Native_GrayMode(void) { return gray_mode; }
int Native_GrayPins(void) { return gray_pin_mask; }
int Native_Keys(uint32_t now,unsigned pressed)
{ milliseconds=now; key_raw=(uint16_t)(65535U&~pressed); return KEY_Scan(0); }
int Native_Count(unsigned i) { return Encoder_Count[i]; }
void Native_EncoderEdge(unsigned line,unsigned gpio)
{
    encoder_gpio=(uint16_t)gpio; exti_pending=1U<<line;
    if(line==0) EXTI0_IRQHandler(); else if(line==1) EXTI1_IRQHandler(); else EXTI15_10_IRQHandler();
}
void Native_SetMask(unsigned x) { irq_mask=x; }
unsigned Native_GetMask(void) { return irq_mask; }
const char *Native_Screen(unsigned row) { return screen[row]; }
/* 校准回归接口仍调用真实速度输出；只由测试输入当前轮速。 */
float Native_CountsPerRev(unsigned i) { return i==0 ? LEFT_COUNTS_PER_REV : RIGHT_COUNTS_PER_REV; }
float Native_BasePWM(unsigned i,int pps)
{
    float rpm=(float)pps*60.0f/(i==0 ? LEFT_COUNTS_PER_REV : RIGHT_COUNTS_PER_REV);
    return i==0 ? LEFT_DRIVE_OFFSET_PWM+LEFT_SPEED_FEEDFORWARD*rpm :
                  RIGHT_DRIVE_OFFSET_PWM+RIGHT_SPEED_FEEDFORWARD*rpm;
}
int Native_SpeedFrame(unsigned i,int target,int rpm,unsigned elapsed)
{
    return SpeedOutput((uint8_t)i,i==0 ? &left_pid : &right_pid,(float)target,(float)rpm,
                       (float)rpm,elapsed,elapsed*.001f,
                       i==0 ? LEFT_SPEED_FEEDFORWARD : RIGHT_SPEED_FEEDFORWARD,
                       i==0 ? LEFT_DRIVE_OFFSET_PWM : RIGHT_DRIVE_OFFSET_PWM);
}
void Native_SetIntegral(unsigned i,int value) { (i==0 ? &left_pid : &right_pid)->integral=(float)value; }
float Native_LoadComp(unsigned i) { return load_compensation[i]; }
int Native_FaultFrame(unsigned i,int pulses,int command,int previous_pwm)
{
    return EncoderFault((uint8_t)i,pulses,(float)pulses*12.0f,(float)command,
                        (int8_t)previous_pwm,TRACK_PERIOD_MS);
}
int Native_Mode(void) { return (int)motion_mode; }
int Native_Hint(void) { return turn_hint; }
int Native_Confidence(void) { return corner_confidence; }
void Native_SetClock(uint32_t now) { milliseconds=last_control=now; }
int Native_RecoveryActive(void) { return recovery_active; }
int Native_RecoveryAttempts(void) { return recovery_attempts; }
unsigned Native_RecoveryStart(void) { return recovery_start_ms; }
float Native_RecoveryAngle(void) { return recovery_total_angle; }
float Native_RecoveryDistance(void) { return recovery_distance; }
float Native_Filtered(unsigned i) { return i==0 ? left_filtered : right_filtered; }
float Native_Integral(unsigned i) { return i==0 ? left_pid.integral : right_pid.integral; }
int Native_StopReason(void) { return stop_reason; }
int Native_OLEDServices(void) { return (int)oled_services; }
void Native_Idle(uint32_t now) { milliseconds=now; Track_Task(); }
void Native_RecoveryAge(uint32_t age) { recovery_start_ms=milliseconds-age; recovery_active=1; }
void Native_RestartScan(int direction)
{
    direction_weak=0; BeginScan(milliseconds,(int8_t)direction,0);
    /* 特意保留调用者的后续动作，验证预算拒绝后不能重新驱动。 */
    SetTargets(TRACK_MOTION_SPIN,18.0f,-18.0f);
    DriveTargets(TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f);
}
void Native_SetEncoder(unsigned i,int value) { Encoder_Count[i]=(int16_t)value; }
void Native_SetTotals(int left,int right) { left_total_counts=left; right_total_counts=right; }
int Native_Total(unsigned i) { return i ? right_total_counts : left_total_counts; }
int Native_DebugError(void) { TrackDebug sample; Track_GetDebug(&sample); return sample.line_error10; }

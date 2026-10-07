/* Host control server.
 * Links the REAL firmware C sources of one variant (Track.c, PID.c, Motor.c,
 * Encoder.c, GraySensor.c, KEY.c, pwm.c) with register-level stubs only.
 * The Python plant sends sensor masks and encoder pulses through stdin and
 * receives one fixed 64-byte record per call through stdout. No control
 * logic is duplicated here; this is not an ARM simulation.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f10x.h"
uint32_t native_pb_pins[16];
static uint16_t gray_raw=255, key_raw=65535, encoder_gpio=65535;
static uint32_t irq_mask, exti_pending;
static unsigned oled_invalid;
#define __get_PRIMASK() irq_mask
#define __disable_irq() (irq_mask=1)
#define __set_PRIMASK(x) (irq_mask=(x))
#include "Encoder.c"
#include "GraySensor.c"
#include "KEY.c"
#include "pwm.c"
#include "Motor.c"
#include "PID.c"
#include "Track.c"

void RCC_APB2PeriphClockCmd(uint32_t p,FunctionalState s) { (void)p;(void)s; }
void RCC_APB1PeriphClockCmd(uint32_t p,FunctionalState s) { (void)p;(void)s; }
void RCC_GetClocksFreq(RCC_ClocksTypeDef *c)
{
    memset(c,0,sizeof(*c)); c->SYSCLK_Frequency=c->HCLK_Frequency=72000000;
    c->PCLK1_Frequency=36000000; c->PCLK2_Frequency=72000000;
}
void GPIO_StructInit(GPIO_InitTypeDef *g) { memset(g,0,sizeof(*g)); }
void GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *g) { (void)p; (void)g; }
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
void TIM_TimeBaseInit(TIM_TypeDef *p,TIM_TimeBaseInitTypeDef *t) { (void)p; (void)t; }
void TIM_OCStructInit(TIM_OCInitTypeDef *p) { memset(p,0,sizeof(*p)); }
void TIM_OC3Init(TIM_TypeDef *p,TIM_OCInitTypeDef *t) { (void)p;(void)t; }
void TIM_OC4Init(TIM_TypeDef *p,TIM_OCInitTypeDef *t) { (void)p;(void)t; }
void TIM_SetCompare3(TIM_TypeDef *p,uint16_t x) { (void)p;(void)x; }
void TIM_SetCompare4(TIM_TypeDef *p,uint16_t x) { (void)p;(void)x; }
void TIM_ClearITPendingBit(TIM_TypeDef *p,uint16_t b) { (void)p;(void)b; }
ITStatus TIM_GetITStatus(TIM_TypeDef *p,uint16_t b) { (void)p;(void)b;return SET; }
void TIM_ITConfig(TIM_TypeDef *p,uint16_t b,FunctionalState s) { (void)p;(void)b;(void)s; }
void TIM_Cmd(TIM_TypeDef *p,FunctionalState s) { (void)p;(void)s; }
void OLED_ShowChar(uint8_t line,uint8_t col,char c)
{ if(line<1 || line>4 || col<1 || col>16) ++oled_invalid; (void)c; }
void OLED_ShowString(uint8_t line,uint8_t col,char *s)
{ while(*s) OLED_ShowChar(line,col++,*s++); }
void OLED_ShowSignedNum(uint8_t line,uint8_t col,int32_t n,uint8_t len)
{
    char buffer[24]; uint32_t m=n<0 ? 0U-(uint32_t)n : (uint32_t)n;
    if(len>10) len=10;
    snprintf(buffer,sizeof(buffer),"%c%0*u",n<0 ? '-' : '+',len,(unsigned)m);
    OLED_ShowString(line,col,buffer);
}
void OLED_Service(uint8_t budget) { if(budget>8) ++oled_invalid; }

/* Variant-specific read-only observation of controller statics. */
#include "probe.inc" /* copied per variant into its build directory */

typedef struct {
    uint32_t now;
    uint8_t state; int8_t left_pwm, right_pwm; uint8_t mask;
    int8_t dir; uint8_t stop_reason, running, mode;
    uint8_t attempts, sweep, phase, flags;
    float left_target, right_target, left_command, right_command;
    float error, angle, left_filtered, right_filtered;
    float approach, goal, aux0, aux1;
} Record;

static void Emit(void)
{
    Record r; unsigned char buffer[64];
    memset(&r,0,sizeof(r));
    r.now=milliseconds; r.state=(uint8_t)state; r.left_pwm=left_pwm; r.right_pwm=right_pwm;
    r.mask=sensor_mask; r.dir=corner_dir; r.running=Track_IsRunning();
    r.left_target=left_target; r.right_target=right_target;
    r.left_command=left_command; r.right_command=right_command;
    r.error=last_error; r.angle=turn_angle; r.left_filtered=left_filtered; r.right_filtered=right_filtered;
    r.approach=approach_mm; r.goal=approach_goal_mm; r.sweep=sweep_number;
    ProbeFill(&r.stop_reason,&r.mode,&r.attempts,&r.phase,&r.flags,&r.aux0,&r.aux1);
    memcpy(buffer,&r,sizeof(buffer));
    fwrite(buffer,1,sizeof(buffer),stdout);
}
static void Reset(void)
{
    memset(native_pb_pins,0,sizeof(native_pb_pins));
    gray_raw=255; key_raw=encoder_gpio=65535; irq_mask=exti_pending=oled_invalid=0;
    Encoder_Count[0]=Encoder_Count[1]=0; Motor_Init(); KEY_Init(); Encoder_Init(); Track_Init();
}
static int ReadExact(void *p,size_t n) { return fread(p,1,n,stdin)==n; }
int main(void)
{
    typedef char record_is_64_bytes[sizeof(Record)==64 ? 1 : -1];
    int command; (void)sizeof(record_is_64_bytes);
    while((command=getchar())!=EOF) {
        if(command=='I') { Reset(); Emit(); }
        else if(command=='S') {
            uint8_t mask;
            if(!ReadExact(&mask,1)) return 2;
            Reset(); gray_raw=(uint16_t)(~mask&255U); Track_Start(); Emit();
        } else if(command=='T') {
            uint32_t now; int16_t left,right; uint8_t mask;
            if(!ReadExact(&now,4) || !ReadExact(&left,2) || !ReadExact(&right,2) || !ReadExact(&mask,1)) return 2;
            Encoder_Count[0]=(int16_t)(LEFT_ENCODER_SIGN*left); Encoder_Count[1]=(int16_t)(RIGHT_ENCODER_SIGN*right);
            gray_raw=(uint16_t)(~mask&255U); milliseconds=now; Track_Task(); Emit();
        } else if(command=='K') {
            uint32_t now; uint8_t key;
            if(!ReadExact(&now,4) || !ReadExact(&key,1)) return 2;
            milliseconds=now; Track_HandleKey(key); Emit();
        } else if(command=='X') {
            /* Extended observation (black box etc.); variant defined. */
            ProbeExtended();
        } else if(command=='Q') break;
        else return 3;
        fflush(stdout);
    }
    return oled_invalid ? 4 : 0;
}

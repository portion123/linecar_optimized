/* Standalone ASan/UBSan entry point.  It executes the real control sources
 * through the native hardware adapter; it is not a behavioural/physics model.
 * Sanitizer runs are intentionally separate from golden outputs and metrics.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "native/harness.c"

static uint32_t random_state=UINT32_C(0x91e10da5);
static uint32_t checked_periods;
static uint32_t trace_reads;

static void Require(int condition,const char *message)
{
    if(!condition) {
        fputs("sanitizer exercise assertion: ",stderr);
        fputs(message,stderr); fputc('\n',stderr); exit(1);
    }
}

static uint32_t Random32(void)
{
    random_state^=random_state<<13;
    random_state^=random_state>>17;
    random_state^=random_state<<5;
    return random_state;
}

static void CheckOutputs(void)
{
    TrackDebug debug;
    memset(&debug,0,sizeof(debug)); Track_GetDebug(&debug);
    Track_GetDebug(NULL); /* Optional observer accepts absent destination. */
    Require(isfinite(Native_LeftTarget()) && isfinite(Native_RightTarget()),"finite requested RPM");
    Require(isfinite(Native_LeftCommand()) && isfinite(Native_RightCommand()),"finite command RPM");
    Require(isfinite(Native_LeftRPM()) && isfinite(Native_RightRPM()),"finite measured RPM");
    Require(isfinite(Native_Error()) && isfinite(Native_Angle()),"finite line error/angle");
    Require(abs(Native_LeftPWM())<=TRACK_MAX_PWM && abs(Native_RightPWM())<=TRACK_MAX_PWM,
            "bounded PWM");
    Require(Native_OLEDInvalid()==0,"valid OLED coordinates/service budget");
    ++checked_periods;
}

static void Step(uint32_t tick,int left,int right,unsigned mask)
{
    Native_Step(tick,left,right,mask); CheckOutputs();
}

static void AllSensorMasks(void)
{
    unsigned mask,period;
    for(mask=0;mask<256U;++mask) {
        Native_Start(0x18U);
        for(period=1;period<=32U;++period)
            Step(period*TRACK_PERIOD_MS,2,2,mask);
        Track_Stop(); CheckOutputs();
    }
}

static void ClockWrapAndSparseFaults(void)
{
    uint32_t tick=UINT32_MAX-19U;
    unsigned period,wheel;
    int sign;
    Native_Start(0x18U); Native_SetClock(tick);
    for(period=0;period<320U;++period) {
        tick+=TRACK_PERIOD_MS; /* Defined uint32_t wrapping is intentional. */
        Step(tick,2,2,period%61U==0 ? 0U : 0x18U);
        if(!Track_IsRunning()) { Native_Sensors(0x18U); Track_Start(); }
    }
    for(wheel=0;wheel<2U;++wheel) for(sign=-1;sign<=1;sign+=2) {
        Native_Reset();
        for(period=0;period<100U;++period) {
            (void)Native_FaultFrame(wheel,period%3U==0 ? -sign : 0,sign*5,sign*9);
            CheckOutputs();
        }
    }
    Native_Start(0x18U); Step(20U,0,0,0x18U);
    Step(20U+TRACK_CONTROL_MAX_GAP_MS+TRACK_PERIOD_MS,0,0,0x18U);
    Require(!Track_IsRunning(),"scheduling fault stops motion");
    Require(Native_LeftPWM()==0 && Native_RightPWM()==0,"fault removes PWM");
}

static void LowSpeedAndReversal(void)
{
    static const int target_rpm[]={5,8,12,18,25}; /* RPM: software input only. */
    unsigned wheel,speed,period;
    int sign,pwm;
    for(wheel=0;wheel<2U;++wheel) for(sign=-1;sign<=1;sign+=2)
        for(speed=0;speed<sizeof(target_rpm)/sizeof(target_rpm[0]);++speed) {
            Native_Reset();
            for(period=0;period<160U;++period) {
                /* Includes a 100 ms no-pulse interval then sparse feedback. */
                int rpm=period<5U || period%4U==0 ? 0 : sign*target_rpm[speed];
                pwm=Native_SpeedFrame(wheel,sign*target_rpm[speed],rpm,TRACK_PERIOD_MS);
                Require(abs(pwm)<=TRACK_MAX_PWM,"speed-loop PWM bound");
                Require(isfinite(Native_Integral(wheel)),"finite speed integral");
                Require(fabsf(Native_Integral(wheel))<=TRACK_SPEED_I_LIMIT+0.0001f,
                        "bounded speed integral");
            }
            (void)Native_SpeedFrame(wheel,0,0,TRACK_PERIOD_MS);
            for(period=0;period<80U;++period) {
                pwm=Native_SpeedFrame(wheel,-sign*20,period<5U ? sign*5 : -sign*20,TRACK_PERIOD_MS);
                Require(abs(pwm)<=TRACK_MAX_PWM,"reversal PWM bound");
                Require(isfinite(Native_Integral(wheel)),"finite reversal integral");
            }
        }
}

static void TraceAndStopRestart(void)
{
    unsigned period;
    uint32_t tick=0;
#if defined(TRACK_TRACE_ENABLE) && TRACK_TRACE_ENABLE
    uint16_t count,index;
    TrackTrace first,last,again;
#endif
    Native_Start(0x18U);
    for(period=0;period<700U;++period) { tick+=TRACK_PERIOD_MS; Step(tick,2,2,0x18U); }
#if defined(TRACK_TRACE_ENABLE) && TRACK_TRACE_ENABLE
    count=Track_GetTraceCount();
    Require(count==TRACK_TRACE_CAPACITY,"trace ring wraps at capacity");
    for(index=0;index<count;++index) {
        memset(&again,0,sizeof(again));
        Require(Track_GetTrace(index,&again)!=0,"valid chronological trace index"); ++trace_reads;
    }
    Require(Track_GetTrace(count,&again)==0,"trace rejects past-end index");
    Require(Track_GetTrace(UINT16_MAX,&again)==0,"trace rejects maximum index");
    Require(Track_GetTrace(0,NULL)==0,"trace rejects null destination");
#endif
    Track_Stop(); Require(!Track_IsRunning(),"explicit STOP disables motion");
#if defined(TRACK_TRACE_ENABLE) && TRACK_TRACE_ENABLE
    count=Track_GetTraceCount();
    memset(&first,0,sizeof(first)); memset(&last,0,sizeof(last));
    Require(count>0,"STOP preserves history");
    Require(Track_GetTrace(0,&first) && Track_GetTrace((uint16_t)(count-1U),&last),
            "STOP history remains readable");
#endif
    for(period=0;period<300U;++period) { tick+=TRACK_PERIOD_MS; Step(tick,0,0,0U); }
    Track_Stop();
    Require(Native_LeftPWM()==0 && Native_RightPWM()==0,"STOP remains off");
#if defined(TRACK_TRACE_ENABLE) && TRACK_TRACE_ENABLE
    Require(Track_GetTraceCount()==count,"STOP freezes count");
    memset(&again,0,sizeof(again)); Require(Track_GetTrace(0,&again)!=0,"frozen oldest exists");
    Require(memcmp(&again,&first,sizeof(first))==0,"STOP freezes oldest record");
    memset(&again,0,sizeof(again)); Require(Track_GetTrace((uint16_t)(count-1U),&again)!=0,
                                          "frozen newest exists");
    Require(memcmp(&again,&last,sizeof(last))==0,"STOP freezes newest record");
#endif
    Native_Sensors(0x18U); Track_Start();
    Require(Track_IsRunning(),"explicit restart enables motion");
    Require(Native_StopReason()==0,"restart clears stop reason");
    for(period=0;period<300U;++period) { tick+=TRACK_PERIOD_MS; Step(tick,2,2,0x18U); }
#if defined(TRACK_TRACE_ENABLE) && TRACK_TRACE_ENABLE
    Require(Track_GetTraceCount()==TRACK_TRACE_CAPACITY,"restarted trace can wrap again");
    Track_ClearTrace(); Require(Track_GetTraceCount()==0,"manual trace clear resets count");
#endif
    Track_Stop();
}

static void RandomLegalPeriods(void)
{
    unsigned period;
    uint32_t tick=UINT32_MAX-9999U;
    Native_Start(0x18U); Native_SetClock(tick);
    for(period=0;period<30000U;++period) {
        uint32_t value=Random32();
        int left=(int)(value%11U)-5, right=(int)((value>>8)%11U)-5;
        unsigned mask=(value>>16)&255U;
        if(!Track_IsRunning()) { Native_Sensors(0x18U); Track_Start(); }
        tick+=TRACK_PERIOD_MS;
        Step(tick,left,right,mask);
        /* Exercise repeated stops, restarts and fixed-size observer buffers. */
        if(period%97U==0) Track_Stop();
        if(period%113U==0) {
            Native_Sensors(0x18U); Track_StartSpeedTest(18,-18);
        }
    }
    Track_Stop();
}

int main(void)
{
    AllSensorMasks(); ClockWrapAndSparseFaults(); LowSpeedAndReversal();
    TraceAndStopRestart(); RandomLegalPeriods();
    printf("{\"status\":\"passed\",\"checked_periods\":%u,\"random_periods\":30000,"
           "\"trace_reads\":%u,\"seed\":%u}\n",(unsigned)checked_periods,
           (unsigned)trace_reads,(unsigned)UINT32_C(0x91e10da5));
    return 0;
}

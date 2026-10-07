/* FIX5 test-only views.  Included after Track.c by the host harness.
 * These functions call production helpers; they contain no alternate controller
 * and are not part of the Keil project or interrupt/control runtime. */

#include <stdint.h>
unsigned Fix5_Enabled(void)
{
    unsigned enabled=0;
#if TRACK_TRACE_ENABLE
    enabled|=1U;
#endif
#if TRACK_SENSOR_FILTER_ENABLE
    enabled|=2U;
#endif
#if TRACK_NONLINEAR_FOLLOW_ENABLE
    enabled|=4U;
#endif
#if TRACK_ERROR_TREND_ENABLE
    enabled|=8U;
#endif
#if TRACK_ADAPTIVE_SPEED_ENABLE
    enabled|=16U;
#endif
#if TRACK_LOW_SPEED_ZONE_ENABLE
    enabled|=32U;
#endif
#if TRACK_TURN_CONTINUITY_ENABLE
    enabled|=64U;
#endif
#if TRACK_ALIGN_TREND_ENABLE
    enabled|=128U;
#endif
    return enabled;
}
float Fix5_Config(unsigned index)
{
    switch(index) {
    case 0: return TRACK_BASE_RPM;
    case 1: return TRACK_TARGET_SLEW_RPM_PER_S*TRACK_PERIOD_MS*0.001f;
    case 2: return TRACK_EDGE_SLEW_RPM_PER_S*TRACK_PERIOD_MS*0.001f;
    case 3: return TRACK_PWM_RISE_PER_TICK;
    case 4: return TRACK_PWM_FALL_PER_TICK;
    case 5: return TRACK_SPEED_I_LIMIT;
    case 6: return (float)TRACK_MAX_PWM;
    case 7: return TRACK_LINE_KP;
    case 8: return TRACK_ALIGN_BASE_RPM;
    case 9: return TRACK_ALIGN_KP;
#if TRACK_ADAPTIVE_SPEED_ENABLE
    case 10: return TRACK_MIN_FOLLOW_BASE_RPM;
    case 11: return TRACK_ADAPTIVE_START_RPM;
    case 12: return TRACK_BASE_RISE_RPM_PER_TICK;
    case 13: return TRACK_BASE_FALL_RPM_PER_TICK;
#endif
#if TRACK_NONLINEAR_FOLLOW_ENABLE
    case 14: return TRACK_CENTER_ZONE;
    case 15: return TRACK_MEDIUM_ZONE;
#endif
#if TRACK_ERROR_TREND_ENABLE
    case 16: return TRACK_ERROR_RATE_LIMIT;
    case 17: return TRACK_ERROR_TREND_SAMPLES;
#endif
#if TRACK_LOW_SPEED_ZONE_ENABLE
    case 18: return SPEED_VERY_LOW_RPM;
    case 19: return SPEED_LOW_RPM;
#endif
#if TRACK_ALIGN_TREND_ENABLE
    case 20: return TRACK_ALIGN_WORSEN_SAMPLES;
    case 21: return TRACK_ALIGN_FLIP_WINDOW;
    case 22: return TRACK_ALIGN_FLIP_REQUIRED;
#endif
    default: return -1.0f;
    }
}
int Fix5_ActualState(void) { return (int)state; }
float Fix5_Correction(void) { return (left_target-right_target)*0.5f; }
float Fix5_RequestedBase(void) { return (left_target+right_target)*0.5f; }
float Fix5_Integral(unsigned wheel) { return (wheel ? &right_pid : &left_pid)->integral; }
float Fix5_Output(unsigned wheel) { return output_pwm[wheel!=0]; }
int Fix5_SpeedFrame(unsigned wheel,float target,float rpm)
{
    return SpeedOutput((uint8_t)wheel,wheel ? &right_pid : &left_pid,target,rpm,rpm,
                       TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,
                       wheel ? RIGHT_SPEED_FEEDFORWARD : LEFT_SPEED_FEEDFORWARD,
                       wheel ? RIGHT_DRIVE_OFFSET_PWM : LEFT_DRIVE_OFFSET_PWM);
}
#if TRACK_TRACE_ENABLE
void Fix5_Stop(unsigned reason) { EnterStop((TrackStopReason)reason); }
unsigned Fix5_TraceCount(void) { return Track_GetTraceCount(); }
unsigned Fix5_TraceSize(void) { return (unsigned)sizeof(TrackTrace); }
int32_t Fix5_TraceField(unsigned index,unsigned field)
{
    TrackTrace sample;
    if(!Track_GetTrace((uint16_t)index,&sample)) return INT32_MIN;
    switch(field) {
    case 0: return (int32_t)sample.timestamp;
    case 1: return sample.state;
    case 2: return sample.sensor_raw;
    case 3: return sample.sensor_filtered;
    case 4: return sample.line_error10;
    case 5: return sample.error_rate10;
    case 6: return sample.left_target10;
    case 7: return sample.right_target10;
    case 8: return sample.left_actual10;
    case 9: return sample.right_actual10;
    case 10: return sample.left_pwm;
    case 11: return sample.right_pwm;
    case 12: return sample.corner_dir;
    case 13: return sample.turn_angle_deg10;
    case 14: return sample.recovery_attempts;
    case 15: return sample.stop_reason;
    case 16: return sample.left_request10;
    case 17: return sample.right_request10;
    default: return INT32_MIN;
    }
}
/* Inject budget boundary values into original fault paths, without calling
 * EnterStop directly or changing the model, limits or control algorithms. */
void Fix5_FaultPath(unsigned reason)
{
    uint32_t tick;
    Native_Start(0x18);
    switch(reason) {
    case TRACK_STOP_USER: Track_HandleKey(KEY0_PRES); break;
    case TRACK_STOP_ENCODER:
        for(tick=TRACK_PERIOD_MS;tick<=2000U && Track_IsRunning();tick+=TRACK_PERIOD_MS)
            Native_Step(tick,0,0,0x18);
        break;
    case TRACK_STOP_PERIOD: Native_Step(TRACK_CONTROL_MAX_GAP_MS+1U,0,0,0x18); break;
    case TRACK_STOP_CONFIRM:
        for(tick=TRACK_PERIOD_MS;tick<=2500U && Track_IsRunning();tick+=TRACK_PERIOD_MS)
            Native_Step(tick,2,2,255);
        break;
    case TRACK_STOP_APPROACH:
        state=CAR_CORNER_APPROACH; approach_mm=CORNER_APPROACH_MAX_MM+1.0f;
        Approach(TRACK_PERIOD_MS,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,0.0f,0.0f);
        break;
    case TRACK_STOP_TURN:
        state=CAR_TURN_FAST; corner_dir=1; phase_started=0;
        TurnCorner(CORNER_TURN_MAX_MS,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,0,0.0f);
        break;
    case TRACK_STOP_ALIGN:
        state=CAR_ALIGN; align_travel_mm=TRACK_ALIGN_MAX_TRAVEL_MM+1.0f;
        Align(TRACK_PERIOD_MS,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,2,0.0f,0.0f);
        break;
    case TRACK_STOP_EXIT:
        state=CAR_EXIT; phase_started=0;
        ExitLine(TRACK_EXIT_MAX_MS,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,2,0.0f,0.0f);
        break;
    case TRACK_STOP_SEARCH:
        state=CAR_SEARCH; phase_started=0; corner_dir=1;
        Scan(TRACK_SEARCH_MAX_MS,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,0,0.0f);
        break;
    case TRACK_STOP_RECOVERY_TIME:
        recovery_active=1; recovery_start_ms=0;
        (void)RecoveryStep(TRACK_RECOVERY_MAX_MS,TRACK_PERIOD_MS,2,0.0f,0.0f,0.0f,0.0f);
        break;
    case TRACK_STOP_RECOVERY_ANGLE:
        recovery_active=1;
        (void)RecoveryStep(TRACK_PERIOD_MS,TRACK_PERIOD_MS,2,0.0f,0.0f,0.0f,TRACK_RECOVERY_MAX_ANGLE_RAD);
        break;
    case TRACK_STOP_RECOVERY_DISTANCE:
        recovery_active=1;
        (void)RecoveryStep(TRACK_PERIOD_MS,TRACK_PERIOD_MS,2,0.0f,
                           TRACK_RECOVERY_MAX_DISTANCE_MM,TRACK_RECOVERY_MAX_DISTANCE_MM,0.0f);
        break;
    case TRACK_STOP_RECOVERY_ATTEMPTS:
        recovery_active=1; recovery_attempts=TRACK_RECOVERY_MAX_ATTEMPTS;
        BeginScan(TRACK_PERIOD_MS,1,0);
        break;
    case TRACK_STOP_NO_DIRECTION:
        state=CAR_SEARCH; corner_dir=0; phase_started=0;
        Scan(TRACK_UNKNOWN_PROBE_MS,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,0,0.0f);
        break;
    case TRACK_STOP_MODE: SetTargets(TRACK_MOTION_FORWARD,-1.0f,1.0f); break;
    case TRACK_STOP_AMBIGUOUS:
        Native_Start(0x81);
        for(tick=TRACK_PERIOD_MS;tick<=1000U && Track_IsRunning();tick+=TRACK_PERIOD_MS)
            Native_Step(tick,2,2,0x81);
        break;
    case TRACK_STOP_FORWARD_TIME:
        Track_StartForward();
        for(tick=TRACK_PERIOD_MS;tick<=TRACK_FORWARD_TEST_MAX_MS && Track_IsRunning();tick+=TRACK_PERIOD_MS)
            Native_Step(tick,2,2,0);
        break;
    case TRACK_STOP_BENCH_TIME:
        Track_StartSpeedTest(18,18);
        for(tick=TRACK_PERIOD_MS;tick<=TRACK_BENCH_MAX_MS && Track_IsRunning();tick+=TRACK_PERIOD_MS)
            Native_Step(tick,2,2,0);
        break;
    default: break;
    }
}
unsigned Fix5_StopReasonNone(void) { return STOP_REASON_NONE; }
unsigned Fix5_StopReasonKey(void) { return STOP_REASON_USER_KEY; }
#endif
#if TRACK_SENSOR_FILTER_ENABLE
unsigned Fix5_Majority(unsigned a,unsigned b,unsigned c)
{ return MajorityMask((uint8_t)a,(uint8_t)b,(uint8_t)c); }
unsigned Fix5_Raw(void) { return sensor_raw; }
unsigned Fix5_Filtered(void) { return sensor_filtered; }
#endif
#if TRACK_NONLINEAR_FOLLOW_ENABLE
float Fix5_Position(float error) { return NonlinearError(error); }
#endif
#if TRACK_ERROR_TREND_ENABLE
void Fix5_TrendReset(float error) { TrendReset(error); }
float Fix5_TrendPush(float error)
{ TrendPush(error,TRACK_PERIOD_MS*0.001f); return error_rate; }
float Fix5_ErrorRate(void) { return error_rate; }
#endif
#if TRACK_ADAPTIVE_SPEED_ENABLE
float Fix5_Base(void) { return adaptive_base_rpm; }
void Fix5_SetBase(float base) { adaptive_base_rpm=base; }
float Fix5_Adaptive(float error,float rate)
{ return AdaptiveStep(error,rate,TRACK_BASE_RPM); }
void Fix5_Sync(unsigned mode,float left,float right,unsigned explicit_base,float base)
{ AdaptiveSync((TrackMotionMode)mode,left,right,(uint8_t)explicit_base,base); }
#endif
void Fix5_LogicReset(void)
{
    Native_Start(0x18);
    last_error=error_filtered=0.0f; PID_Reset(&line_pid);
#if TRACK_ERROR_TREND_ENABLE
    TrendReset(0.0f);
#endif
}
void Fix5_LogicFollow(float error)
{
    milliseconds+=TRACK_PERIOD_MS;
#if TRACK_ERROR_TREND_ENABLE
    TrendPush(error,TRACK_PERIOD_MS*0.001f);
#endif
    FollowLine(milliseconds,error,TRACK_PERIOD_MS*0.001f,TRACK_BASE_RPM);
    DriveTargets(TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f);
}
#if TRACK_LOW_SPEED_ZONE_ENABLE
float Fix5_GainScale(float target,unsigned integral)
{ float kp,ki; SpeedGainScale(target,&kp,&ki); return integral ? ki : kp; }
#endif
#if TRACK_TURN_CONTINUITY_ENABLE
float Fix5_Turn(float degrees)
{ return TurnRequested(degrees*0.01745329252f); }
float Fix5_TurnCommand(float command,float target)
{ return SlewTarget(command,target,TRACK_TURN_ACCEL_RPM,TRACK_TURN_DECEL_RPM); }
void Fix5_TurnFrame(float degrees)
{
    Native_Start(0x18); state=CAR_TURN_FAST; corner_dir=1;
    phase_started=0; milliseconds=TRACK_PERIOD_MS;
    turn_angle=-degrees*0.01745329252f;
    TurnCorner(milliseconds,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,0,0.0f);
}
#endif
#if TRACK_ALIGN_TREND_ENABLE
static float fix5_align_base,fix5_align_gain;
void Fix5_AlignReset(float error)
{ AlignEnhanceReset(error); fix5_align_base=TRACK_ALIGN_BASE_RPM; fix5_align_gain=TRACK_ALIGN_KP; }
void Fix5_AlignFrame(float error)
{
    fix5_align_base=TRACK_ALIGN_BASE_RPM; fix5_align_gain=TRACK_ALIGN_KP;
    AlignEnhance(error,&fix5_align_base,&fix5_align_gain);
}
float Fix5_AlignBase(void) { return fix5_align_base; }
float Fix5_AlignGain(void) { return fix5_align_gain; }
void Fix5_AlignControlledReset(float error)
{ Native_Start(0x18); BeginAlign(milliseconds,error); }
void Fix5_AlignControlledFrame(float error)
{
    milliseconds+=TRACK_PERIOD_MS; sensor_raw_error=error;
    Align(milliseconds,TRACK_PERIOD_MS,TRACK_PERIOD_MS*0.001f,2,error,0.0f);
}
#endif

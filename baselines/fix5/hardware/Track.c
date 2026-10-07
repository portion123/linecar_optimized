/* 循迹总控制：探头求位置 → 位置 PD 分配轮速 → 左右速度 PI 生成 PWM。
 * 普通弯道连续纠偏；直角先推进轮轴，再旋转找下一条边。
 * 尺寸、编码器计数和方向必须匹配实车，软件测试不能替代实车调参。
 */
#include "stm32f10x.h"
#include "Track.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "OLED.h"
#include "CarConfig.h"
#include "GraySensor.h"
#include "KEY.h"

/* 状态像分步交通指挥，不能一读全白就从直行跳到旋转。 */
typedef TrackState CarState;
static CarState state=CAR_READY;
static volatile uint32_t milliseconds;
static uint32_t last_control, last_display, phase_started,last_oled_service;
static uint32_t control_elapsed_ms,max_control_elapsed_ms;
static TrackStopReason stop_reason;
#if TRACK_DEBUG
volatile TrackDebug track_debug;
static uint32_t last_debug;
#endif
static float last_error, error_filtered, observed_error, left_rpm, right_rpm;
static float left_filtered, right_filtered, left_target, right_target;
static float left_command, right_command;
static int8_t left_pwm, right_pwm;
static uint8_t sensor_mask, normal_line_width=2, display_row;
static PID_TypeDef left_pid, right_pid, line_pid;
static float load_compensation[2];
static float direction_load[2][2], output_pwm[2];
static uint32_t reverse_wait_ms[2];
static uint32_t boost_ramp_ms[2];
static TrackMotionMode motion_mode=TRACK_MOTION_FORWARD;
static int32_t direction_window[2][ENCODER_DIRECTION_WINDOW];
static uint8_t direction_index[2];
static int8_t fault_sign[2];
static uint32_t direction_grace_ms[2], direction_time_ms[2];
static int32_t rpm_window[2][TRACK_RPM_WINDOW_SAMPLES];
static uint32_t rpm_window_ms[TRACK_RPM_WINDOW_SAMPLES];
static uint8_t rpm_index;
/* 每个车轮独立记录故障和助推，避免左右状态互相干扰。 */
static uint32_t no_pulse_ms[2], reverse_ms[2], still_ms[2], boost_left_ms[2], moving_ms[2];
static uint8_t boost_used[2];
static int8_t drive_sign[2];
static int32_t left_total_counts, right_total_counts;
/* 记录横向宽线的进入/离开距离，估算其中心，不使用固定转弯延时。 */
static uint8_t wide_active, wide_strong, wide_peak_count, corner_armed=1, sweep_number, center_samples;
static uint32_t wide_ms, wide_last_seen, lost_ms, centered_ms, wide_gap_ms;
static float wide_travel_mm, wide_last_mm, approach_mm, approach_goal_mm, rearm_mm, turn_angle;
static int8_t wide_dir, corner_dir, last_turn_dir;
static uint32_t wide_center_ms, wide_tail_ms, side_ms, hint_seen;
static float wide_center_mm, side_error;
static int8_t side_candidate, side_observed, turn_hint;
static uint8_t steering_urgent, edge_align_active;
static uint32_t edge_ms;
static int8_t edge_dir;
static float edge_best_error;
static uint32_t width_learn_ms;
static uint8_t width_candidate,corner_confidence,direction_weak,alignment_from_corner;
static int8_t corner_vote,exit_dir;
static uint32_t wide_started,line_seen,last_turn_seen,align_center_ms,exit_center_ms;
static float approach_yaw,align_angle,align_travel_mm,exit_travel_mm,acquire_error;
static uint8_t acquire_valid,search_near_exit;
static uint8_t line_ambiguous;
static uint32_t ambiguous_ms;
static uint8_t recovery_active,recovery_attempts,search_stage;
static uint32_t recovery_start_ms,recovery_stable_ms;
static float recovery_total_angle,recovery_distance,recovery_stable_mm;
static uint32_t approach_sync_ms;
static uint8_t approach_synced;
/* Raw/filtered and trend fields are observations until their switches are on. */
static uint8_t sensor_raw,sensor_filtered;
static float error_rate;
#if TRACK_ERROR_TREND_ENABLE || TRACK_ADAPTIVE_SPEED_ENABLE || TRACK_ERROR_PREDICT_ENABLE
static float error_history[TRACK_ERROR_TREND_SAMPLES]; /* sensor units */
static float error_history_dt[TRACK_ERROR_TREND_SAMPLES]; /* seconds */
static uint8_t error_history_index,error_history_count;
#endif
static float sensor_raw_error;
#if TRACK_ALIGN_TREND_ENABLE
static float align_previous_magnitude; /* sensor position units */
static uint8_t align_worsen_count,align_worsen_active;
static uint8_t align_flip_history[TRACK_ALIGN_FLIP_WINDOW],align_flip_index,align_flip_count;
static int8_t align_previous_sign;
#endif
#if TRACK_ADAPTIVE_SPEED_ENABLE
static float adaptive_base_rpm; /* requested forward base RPM, never actual RPM */
#endif
#if TRACK_NONLINEAR_FOLLOW_ENABLE
static float nonlinear_error_filtered;
#endif
#if TRACK_SENSOR_FILTER_ENABLE
static uint8_t sensor_history[3],sensor_history_index,sensor_history_ready;
static uint8_t MajorityMask(uint8_t a,uint8_t b,uint8_t c)
{ return (uint8_t)((a&b)|(a&c)|(b&c)); }
#endif
#if TRACK_TRACE_ENABLE
static TrackTrace trace_buffer[TRACK_TRACE_CAPACITY];
static uint16_t trace_write,trace_count;
static uint8_t trace_frozen,trace_in_control,trace_freeze_pending;
#endif
#if TRACK_TRACE_ENABLE
static void EnterStop(TrackStopReason why);
#endif
static int16_t Debug10(float value);

void Track_ClearTrace(void)
{
#if TRACK_TRACE_ENABLE
    trace_write=trace_count=0;
#endif
}
uint16_t Track_GetTraceCount(void)
{
#if TRACK_TRACE_ENABLE
    return trace_count;
#else
    return 0;
#endif
}
uint8_t Track_GetTrace(uint16_t oldest_index,TrackTrace *sample)
{
#if TRACK_TRACE_ENABLE
    uint16_t start;
    if(!sample || oldest_index>=trace_count) return 0;
    start=(uint16_t)((trace_write+TRACK_TRACE_CAPACITY-trace_count)%TRACK_TRACE_CAPACITY);
    *sample=trace_buffer[(start+oldest_index)%TRACK_TRACE_CAPACITY]; return 1;
#else
    (void)oldest_index; (void)sample; return 0;
#endif
}
#if TRACK_TRACE_ENABLE
static void TraceFreeze(void) { trace_frozen=1; trace_freeze_pending=0; }
static void TraceRestart(void)
{
    Track_ClearTrace(); trace_frozen=trace_freeze_pending=0;
}
/* Called exactly once at the end of a scheduled main-loop control task.
 * A fault in this task is included as the final record, then frozen. Key stop
 * outside a task freezes immediately; its reason remains in Track_GetDebug. */
static void TraceWrite(uint32_t now)
{
    TrackTrace *sample;
    if(trace_frozen) return;
    sample=&trace_buffer[trace_write];
    sample->timestamp=now; sample->state=(uint8_t)state;
    sample->sensor_raw=sensor_raw; sample->sensor_filtered=sensor_filtered;
    sample->line_error10=Debug10(observed_error); sample->error_rate10=Debug10(error_rate);
    sample->left_target10=Debug10(left_command); sample->right_target10=Debug10(right_command);
    sample->left_request10=Debug10(left_target); sample->right_request10=Debug10(right_target);
    sample->left_actual10=Debug10(left_filtered); sample->right_actual10=Debug10(right_filtered);
    sample->left_pwm=left_pwm; sample->right_pwm=right_pwm; sample->corner_dir=corner_dir;
    sample->turn_angle_deg10=Debug10(turn_angle*57.2957795f);
    sample->recovery_attempts=recovery_attempts; sample->stop_reason=(uint8_t)stop_reason;
    trace_write=(uint16_t)((trace_write+1U)%TRACK_TRACE_CAPACITY);
    if(trace_count<TRACK_TRACE_CAPACITY) ++trace_count;
    if(trace_freeze_pending) TraceFreeze();
}
#endif

uint32_t Track_Now(void) { return milliseconds; }
/* 每 1 ms 只记时间。中断中不刷屏、不算 PID，避免耽误编码器。 */
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET) {
        TIM_ClearITPendingBit(TIM3,TIM_IT_Update); ++milliseconds;
    }
}
static float Abs(float x) { return x<0.0f ? -x : x; }
static float Limit(float x,float lo,float hi)
{
    if(x<lo) return lo;
    if(x>hi) return hi;
    return x;
}
#if TRACK_ERROR_TREND_ENABLE || TRACK_ADAPTIVE_SPEED_ENABLE || TRACK_ERROR_PREDICT_ENABLE
static void TrendReset(float error)
{
    uint8_t i;
    for(i=0;i<TRACK_ERROR_TREND_SAMPLES;++i) { error_history[i]=error; error_history_dt[i]=0.0f; }
    error_history_index=0; error_history_count=1; error_rate=0.0f;
}
static void TrendPush(float error,float dt)
{
    uint8_t oldest,i,index;
    float span=0.0f;
    if(dt<=0.0f) { TrendReset(error); return; }
    error_history[error_history_index]=error; error_history_dt[error_history_index]=dt;
    error_history_index=(uint8_t)((error_history_index+1U)%TRACK_ERROR_TREND_SAMPLES);
    if(error_history_count<TRACK_ERROR_TREND_SAMPLES) ++error_history_count;
    oldest=(uint8_t)((error_history_index+TRACK_ERROR_TREND_SAMPLES-error_history_count)%TRACK_ERROR_TREND_SAMPLES);
    for(i=1;i<error_history_count;++i) {
        index=(uint8_t)((error_history_index+TRACK_ERROR_TREND_SAMPLES-i)%TRACK_ERROR_TREND_SAMPLES);
        span+=error_history_dt[index];
    }
    error_rate=span>0.0f ? Limit((error-error_history[oldest])/span,
                              -(float)TRACK_ERROR_RATE_LIMIT,(float)TRACK_ERROR_RATE_LIMIT) : 0.0f;
}
#endif
#if TRACK_ADAPTIVE_SPEED_ENABLE
static void AdaptiveSync(TrackMotionMode mode,float left,float right,uint8_t explicit_base,float base)
{
    if(explicit_base && base>=0.0f) adaptive_base_rpm=Limit(base,0.0f,(float)TRACK_MAX_BASE_RPM);
    else if(mode==TRACK_MOTION_FORWARD && left>0.0f && right>0.0f)
        adaptive_base_rpm=Limit((left+right)*0.5f,0.0f,(float)TRACK_MAX_BASE_RPM);
    /* Spin / one-wheel pivot retains the last valid forward base. */
}
static float AdaptiveStep(float error,float rate,float cap)
{
    float outward=error*rate>0.0f ? Abs(rate) : 0.0f;
    float goal=(float)TRACK_MAX_BASE_RPM-(float)TRACK_ADAPTIVE_K_ERR_PERCENT*0.01f*Abs(error)-
               (float)TRACK_ADAPTIVE_K_RATE_PERCENT*0.01f*outward;
    float upper=Limit(cap,0.0f,(float)TRACK_MAX_BASE_RPM);
    float lower=upper<(float)TRACK_MIN_FOLLOW_BASE_RPM ? upper : (float)TRACK_MIN_FOLLOW_BASE_RPM;
    goal=Limit(goal,lower,upper);
    adaptive_base_rpm+=Limit(goal-adaptive_base_rpm,-(float)TRACK_BASE_FALL_RPM_PER_TICK,
                            (float)TRACK_BASE_RISE_RPM_PER_TICK);
    adaptive_base_rpm=Limit(adaptive_base_rpm,0.0f,upper);
    return adaptive_base_rpm; /* no lower clamp: low EXIT climbs naturally */
}
#endif
#if TRACK_ALIGN_TREND_ENABLE
static void AlignEnhanceReset(float error)
{
    uint8_t i;
    align_previous_magnitude=Abs(error); align_worsen_count=align_worsen_active=0;
    align_flip_index=align_flip_count=0;
    align_previous_sign=error>TRACK_ALIGN_FLIP_ERROR ? 1 : (error<-TRACK_ALIGN_FLIP_ERROR ? -1 : 0);
    for(i=0;i<TRACK_ALIGN_FLIP_WINDOW;++i) align_flip_history[i]=0;
}
static void AlignEnhance(float error,float *base,float *gain)
{
    float magnitude=Abs(error),epsilon=(float)TRACK_ALIGN_WORSEN_EPS_PERCENT*0.01f;
    int8_t sign=error>TRACK_ALIGN_FLIP_ERROR ? 1 : (error<-TRACK_ALIGN_FLIP_ERROR ? -1 : 0);
    uint8_t flip=sign && align_previous_sign && sign!=align_previous_sign;
    if(magnitude>align_previous_magnitude+epsilon) {
        if(align_worsen_count<TRACK_ALIGN_WORSEN_SAMPLES) ++align_worsen_count;
        if(align_worsen_count>=TRACK_ALIGN_WORSEN_SAMPLES) align_worsen_active=1;
    } else {
        align_worsen_count=0;
        if(magnitude<align_previous_magnitude-epsilon) align_worsen_active=0;
    }
    align_previous_magnitude=magnitude;
    align_flip_count=(uint8_t)(align_flip_count-align_flip_history[align_flip_index]+flip);
    align_flip_history[align_flip_index]=flip;
    align_flip_index=(uint8_t)((align_flip_index+1U)%TRACK_ALIGN_FLIP_WINDOW);
    if(sign) align_previous_sign=sign;
    if(align_worsen_active) {
        if(*base>(float)TRACK_ALIGN_WORSEN_BASE_RPM) *base=(float)TRACK_ALIGN_WORSEN_BASE_RPM;
        *gain*=(float)TRACK_ALIGN_WORSEN_GAIN_PERCENT*0.01f;
    }
    if(align_flip_count>=TRACK_ALIGN_FLIP_REQUIRED)
        *gain*=(float)TRACK_ALIGN_SWING_GAIN_PERCENT*0.01f;
}
#endif
static uint8_t Bits(uint8_t mask)
{
    uint8_t n=0;
    while(mask) { n+=mask&1U; mask>>=1; }
    return n;
}
static uint8_t Contiguous(uint8_t mask)
{
    if(!mask) return 0;
    while(!(mask&1U)) mask>>=1;
    return (mask&(uint8_t)(mask+1U))==0;
}
/* 步骤 1：逐块求黑区位置，不把分离黑区平均成虚假中线。
 * 同时保留“中线旁边只有一侧出现黑区”的线索，不能永久只选原来的中线。
 */
static uint8_t DecodeLine(uint8_t mask,float *error)
{
    uint8_t i=0,first,n,has_center=0,has_left=0,has_right=0;
    float left_position=0.0f,right_position=0.0f;
    float position,distance,best=100.0f;
    sensor_mask=mask; *error=0.0f; side_observed=0; line_ambiguous=0;
    while(i<8) {
        if(!(sensor_mask&(1U<<i))) { ++i; continue; }
        first=i; n=0;
        while(i<8 && (sensor_mask&(1U<<i))) { ++n; ++i; }
        position=(float)(2*first+n-1)-7.0f;
        if(first<=4 && i>3) has_center=1;
        if(i<=3) { has_left=1; left_position=position; }
        if(first>=5) { has_right=1; right_position=position; }
        distance=Abs(position-last_error);
        if(distance<best) { best=distance; *error=position; line_ambiguous=0; }
        else if(distance==best && position*(*error)<0.0f) {
            *error=last_error; line_ambiguous=1;
        }
    }
    if(has_center && has_left!=has_right) {
        side_observed=has_left ? -1 : 1;
        side_error=has_left ? left_position : right_position;
    }
    observed_error=*error;
    return Bits(sensor_mask);
}
/* GPIO sampled once per 20 ms. Startup seeds all frames from its actual read;
 * a real edge therefore changes majority after one following scheduled frame. */
static uint8_t ReadLine(float *error)
{
    uint8_t count;
    sensor_raw=Gray_Read(); sensor_filtered=sensor_raw;
#if TRACK_SENSOR_FILTER_ENABLE
    if(!sensor_history_ready) {
        sensor_history[0]=sensor_history[1]=sensor_history[2]=sensor_raw;
        sensor_history_index=0; sensor_history_ready=1;
    } else {
        sensor_history[sensor_history_index]=sensor_raw;
        sensor_history_index=(uint8_t)((sensor_history_index+1U)%TRACK_SENSOR_FILTER_FRAMES);
    }
    sensor_filtered=MajorityMask(sensor_history[0],sensor_history[1],sensor_history[2]);
#endif
    count=DecodeLine(sensor_raw,&sensor_raw_error);
#if TRACK_SENSOR_FILTER_ENABLE
    (void)DecodeLine(sensor_filtered,error);
    sensor_mask=sensor_raw; /* Geometry, loss/acquisition and safety keep raw. */
#else
    *error=sensor_raw_error;
#endif
    return count;
}
static void ResolveSide(uint32_t elapsed,float *error)
{
    if(!side_observed || elapsed>2U*TRACK_PERIOD_MS) {
        side_candidate=0; side_ms=0; return;
    }
    if(side_candidate!=side_observed) { side_candidate=side_observed; side_ms=0; }
    if(side_ms<TRACK_SIDE_CONFIRM_MS) side_ms+=elapsed;
    if(side_ms>=TRACK_SIDE_CONFIRM_MS) *error=side_error;
}
static uint8_t Centered(uint8_t count,float error)
{
    return count && count<=5 && (sensor_mask&0x18U) && !(sensor_mask&0x81U) &&
           Abs(error)<=TRACK_CENTER_ERROR && Contiguous(sensor_mask);
}
static void ClearWide(void)
{
    wide_active=wide_strong=wide_peak_count=0; wide_ms=0; wide_dir=0; wide_travel_mm=wide_last_mm=0.0f;
    wide_center_ms=wide_tail_ms=wide_gap_ms=0; wide_center_mm=0.0f;
    corner_confidence=0; corner_vote=0; wide_started=wide_last_seen=0;
}
/* 只保存已运动且未明显超速时的本方向负载补偿，不保存误差和微分。
 * 正反转均按速度大小控制，补偿也按大小使用；手动停车会全部清空。
 */
static void SaveLoad(uint8_t wheel,PID_TypeDef *pid)
{
    uint8_t direction=drive_sign[wheel]<0 ? 1U : 0U;
    if(drive_sign[wheel] && pid->integral>0.0f &&
       drive_sign[wheel]*pid->actual>=TRACK_LOAD_LEARN_MIN_RPM &&
       Abs(pid->actual)<=Abs(pid->target)+TRACK_SPEED_COAST_MARGIN) {
        direction_load[wheel][direction]=Limit(load_compensation[wheel]+pid->integral,
                                               0.0f,TRACK_LOAD_COMP_MAX_PWM);
        load_compensation[wheel]=direction_load[wheel][direction];
        pid->integral=0.0f; /* 同一份积分不能重复叠加到补偿。 */
    }
}
static void ResetFeedback(void)
{
    uint8_t i,w;
    rpm_index=0;
    for(i=0;i<TRACK_RPM_WINDOW_SAMPLES;++i) {
        rpm_window_ms[i]=0;
        rpm_window[0][i]=rpm_window[1][i]=0;
    }
    for(w=0;w<2;++w) {
        no_pulse_ms[w]=reverse_ms[w]=0;
        fault_sign[w]=0; direction_index[w]=0;
        direction_grace_ms[w]=direction_time_ms[w]=0;
        for(i=0;i<ENCODER_DIRECTION_WINDOW;++i) direction_window[w][i]=0;
    }
}
/* 只在初始化/人工停车调用；运动状态切换仅重置位置环，保留速度连续性。 */
static void ResetPID(void)
{
    uint8_t w;
    PID_Init(&left_pid,TRACK_SPEED_KP,TRACK_SPEED_KI,TRACK_SPEED_KD,TRACK_SPEED_I_LIMIT,TRACK_SPEED_D_TAU);
    PID_Init(&right_pid,TRACK_SPEED_KP,TRACK_SPEED_KI,TRACK_SPEED_KD,TRACK_SPEED_I_LIMIT,TRACK_SPEED_D_TAU);
#if TRACK_ERROR_PREDICT_ENABLE
    /* Experimental P lookahead REPLACES the old D contribution, never adds a
     * second derivative contribution to it. All-off retains the original PD. */
    PID_Init(&line_pid,TRACK_LINE_KP,TRACK_LINE_KI,0.0f,TRACK_LINE_I_LIMIT,TRACK_LINE_D_TAU);
#else
    PID_Init(&line_pid,TRACK_LINE_KP,TRACK_LINE_KI,TRACK_LINE_KD,TRACK_LINE_I_LIMIT,TRACK_LINE_D_TAU);
#endif
    left_filtered=left_rpm; right_filtered=right_rpm;
    left_target=right_target=left_command=right_command=0.0f;
    for(w=0;w<2;++w) {
        still_ms[w]=boost_left_ms[w]=moving_ms[w]=reverse_wait_ms[w]=0;
        boost_used[w]=0; drive_sign[w]=0; output_pwm[w]=0.0f;
        boost_ramp_ms[w]=0;
    }
    steering_urgent=0; motion_mode=TRACK_MOTION_FORWARD; ResetFeedback();
}
static void SetPWM(int8_t left,int8_t right)
{
    left_pwm=left; right_pwm=right;
#if DRIVE_PAIRS_SWAPPED
    Motor_SetLeftspeed(right); Motor_SetRightspeed(left);
#else
    Motor_SetLeftspeed(left); Motor_SetRightspeed(right);
#endif
}
uint8_t Track_IsRunning(void)
{
    return state==CAR_TRACK || state==CAR_CORNER_CONFIRM || state==CAR_SEARCH || state==CAR_FORWARD ||
           state==CAR_CORNER_APPROACH || state==CAR_TURN_FAST || state==CAR_TURN_SLOW || state==CAR_ALIGN || state==CAR_EXIT || state==CAR_SPEED_TEST;
}
static void ResetRuntime(void)
{
    SetPWM(0,0); state=CAR_READY; stop_reason=TRACK_STOP_USER; last_error=error_filtered=observed_error=0.0f;
    load_compensation[0]=load_compensation[1]=0.0f;
    direction_load[0][0]=direction_load[0][1]=direction_load[1][0]=direction_load[1][1]=0.0f;
    ClearWide(); lost_ms=centered_ms=0; approach_mm=approach_goal_mm=turn_angle=rearm_mm=0.0f;
    corner_dir=last_turn_dir=0; sweep_number=center_samples=0;
    side_candidate=side_observed=turn_hint=0; side_ms=hint_seen=0;
    edge_ms=0; edge_dir=0;
    edge_align_active=0;
    edge_best_error=0.0f;
    width_learn_ms=0; width_candidate=0;
    corner_armed=1; normal_line_width=2;
    direction_weak=alignment_from_corner=acquire_valid=search_near_exit=0;
    exit_dir=0; line_seen=last_turn_seen=align_center_ms=exit_center_ms=0;
    approach_yaw=align_angle=align_travel_mm=exit_travel_mm=acquire_error=0.0f;
    ambiguous_ms=0; line_ambiguous=0;
    recovery_active=recovery_attempts=search_stage=0;
    recovery_start_ms=recovery_stable_ms=0;
    recovery_total_angle=recovery_distance=recovery_stable_mm=0.0f;
    approach_sync_ms=0; approach_synced=0;
    ResetPID(); error_rate=0.0f; sensor_raw_error=0.0f;
#if TRACK_ALIGN_TREND_ENABLE
    AlignEnhanceReset(0.0f);
#endif
#if TRACK_ADAPTIVE_SPEED_ENABLE
    adaptive_base_rpm=(float)TRACK_ADAPTIVE_START_RPM;
#endif
#if TRACK_ERROR_TREND_ENABLE || TRACK_ADAPTIVE_SPEED_ENABLE || TRACK_ERROR_PREDICT_ENABLE
    TrendReset(0.0f);
#endif
#if TRACK_NONLINEAR_FOLLOW_ENABLE
    nonlinear_error_filtered=0.0f;
#endif
#if TRACK_SENSOR_FILTER_ENABLE
    sensor_history_ready=sensor_history_index=0;
#endif
#if TRACK_TRACE_ENABLE
    stop_reason=STOP_REASON_NONE; TraceRestart();
#endif
}
void Track_Stop(void)
{
#if TRACK_TRACE_ENABLE
    EnterStop(STOP_REASON_USER_KEY);
#else
    ResetRuntime();
#endif
}
/* 安全停机保留方向/角度/距离等现场，人工Stop/Start再清空。 */
static void Halt(CarState reason)
{
#if TRACK_TRACE_ENABLE
    TrackStopReason why;
    if(state==CAR_STOP) return;
    if(reason==CAR_ENCODER_FAULT) why=TRACK_STOP_ENCODER;
    else if(reason==CAR_CONTROL_FAULT) why=TRACK_STOP_PERIOD;
    else switch(state) {
        case CAR_CORNER_CONFIRM: why=TRACK_STOP_CONFIRM; break;
        case CAR_CORNER_APPROACH: why=TRACK_STOP_APPROACH; break;
        case CAR_TURN_FAST: case CAR_TURN_SLOW: why=TRACK_STOP_TURN; break;
        case CAR_ALIGN: why=TRACK_STOP_ALIGN; break;
        case CAR_EXIT: why=TRACK_STOP_EXIT; break;
        default: why=TRACK_STOP_SEARCH; break;
    }
    EnterStop(why);
#else
    if(reason==CAR_ENCODER_FAULT) stop_reason=TRACK_STOP_ENCODER;
    else if(reason==CAR_CONTROL_FAULT) stop_reason=TRACK_STOP_PERIOD;
    else {
        switch(state) {
            case CAR_CORNER_CONFIRM: stop_reason=TRACK_STOP_CONFIRM; break;
            case CAR_CORNER_APPROACH: stop_reason=TRACK_STOP_APPROACH; break;
            case CAR_TURN_FAST: case CAR_TURN_SLOW: stop_reason=TRACK_STOP_TURN; break;
            case CAR_ALIGN: stop_reason=TRACK_STOP_ALIGN; break;
            case CAR_EXIT: stop_reason=TRACK_STOP_EXIT; break;
            default: stop_reason=TRACK_STOP_SEARCH; break;
        }
    }
    SetPWM(0,0); state=reason;
    left_target=right_target=left_command=right_command=0.0f;
    PID_Reset(&left_pid); PID_Reset(&right_pid); PID_Reset(&line_pid);
#endif
}
#if TRACK_TRACE_ENABLE
static void EnterStop(TrackStopReason why)
{
    if(state==CAR_STOP) return; /* Idempotent, retain the first cause. */
    if(why==STOP_REASON_NONE) why=TRACK_STOP_MODE;
    stop_reason=why;
    if(why==STOP_REASON_USER_KEY) {
        /* Preserve the original manual-stop calibration contract, while fault
         * stops keep their load evidence and the trace already holds history. */
        load_compensation[0]=load_compensation[1]=0.0f;
        direction_load[0][0]=direction_load[0][1]=direction_load[1][0]=direction_load[1][1]=0.0f;
    }
    left_target=right_target=left_command=right_command=0.0f;
    SetPWM(0,0); PID_Reset(&left_pid); PID_Reset(&right_pid); PID_Reset(&line_pid);
    state=CAR_STOP;
    if(trace_in_control) trace_freeze_pending=1; else TraceFreeze();
}
#endif
static void HaltReason(TrackStopReason why)
{
#if TRACK_TRACE_ENABLE
    EnterStop(why);
#else
    Halt(CAR_STOP); stop_reason=why;
#endif
}
/* KEY5 人工直行检查不依赖黑线和速度闭环，保留固定 30% PWM。
 * 计数在开始时清零，方便核对两轮实际向前时的编码器正负号。
 */
void Track_StartForward(void)
{
    ResetRuntime(); Encoder_Clear(1); Encoder_Clear(2);
    left_rpm=right_rpm=0.0f; ResetPID(); left_total_counts=right_total_counts=0;
    last_control=phase_started=Track_Now(); stop_reason=TRACK_STOP_NONE; state=CAR_FORWARD; SetPWM(FORWARD_PWM,FORWARD_PWM);
}
void Track_Start(void)
{
    float error; uint8_t count;
    ResetRuntime(); count=ReadLine(&error);
    if(!count) { state=CAR_NO_LINE; return; }
    if(Centered(count,error) && count>normal_line_width) normal_line_width=count;
    Encoder_Clear(1); Encoder_Clear(2); left_rpm=right_rpm=0.0f; ResetPID();
    left_total_counts=right_total_counts=0; last_error=error_filtered=error;
#if TRACK_NONLINEAR_FOLLOW_ENABLE
    nonlinear_error_filtered=error;
#endif
#if TRACK_ERROR_TREND_ENABLE || TRACK_ADAPTIVE_SPEED_ENABLE || TRACK_ERROR_PREDICT_ENABLE
    TrendReset(error);
#endif
    line_seen=Track_Now();
    if(Contiguous(sensor_mask) && count<=normal_line_width+1U && Abs(error)>=TRACK_HINT_ERROR) {
        turn_hint=error<0.0f ? -1 : 1; hint_seen=line_seen;
    }
    last_control=Track_Now(); stop_reason=TRACK_STOP_NONE; state=CAR_TRACK;
}
void Track_StartSpeedTest(int16_t left_speed,int16_t right_speed)
{
    ResetRuntime();
    if(Abs((float)left_speed)>TRACK_BENCH_MAX_RPM || Abs((float)right_speed)>TRACK_BENCH_MAX_RPM ||
       (left_speed<0 && right_speed<0)) { HaltReason(TRACK_STOP_MODE); return; }
    Encoder_Clear(1); Encoder_Clear(2); left_rpm=right_rpm=0.0f; ResetPID();
    left_total_counts=right_total_counts=0; phase_started=last_control=Track_Now();
    state=CAR_SPEED_TEST; stop_reason=TRACK_STOP_NONE;
    motion_mode=left_speed<0 || right_speed<0 ? TRACK_MOTION_SPIN :
                (!left_speed || !right_speed ? TRACK_MOTION_PIVOT : TRACK_MOTION_FORWARD);
    left_target=(float)left_speed; right_target=(float)right_speed;
}
void Track_HandleKey(uint8_t key)
{
    if(key==KEY0_PRES || key==KEY2_PRES) Track_Stop();
    else if(key==KEY1_PRES || key==KEY5_PRES) {
        if(Track_IsRunning()) Track_Stop();
        else if(key==KEY5_PRES) Track_StartForward();
        else Track_Start();
    }
}
/* 步骤 3：左右轮各自用“目标速度 - 实际速度”算 PI。
 * 反转也先转换为速度大小计算，最后恢复电机方向符号。
 */
#if TRACK_LOW_SPEED_ZONE_ENABLE
/* Dimensionless PI scales; target is RPM. Piecewise linear at 0/VERY_LOW/LOW.
 * Integral is already a PWM contribution, so changing Ki neither rescales nor
 * clears learned integral. Reversal/zero/PWM-slew guards below remain in charge.
 */
static void SpeedGainScale(float target,float *kp,float *ki)
{
    float mix,kp_percent,ki_percent;
    target=Abs(target);
    if(target<(float)SPEED_VERY_LOW_RPM) {
        mix=target/(float)SPEED_VERY_LOW_RPM;
        kp_percent=(float)SPEED_KP_ZERO_PERCENT+
                   (float)(SPEED_KP_VERY_LOW_PERCENT-SPEED_KP_ZERO_PERCENT)*mix;
        ki_percent=(float)SPEED_KI_ZERO_PERCENT+
                   (float)(SPEED_KI_VERY_LOW_PERCENT-SPEED_KI_ZERO_PERCENT)*mix;
    } else if(target<(float)SPEED_LOW_RPM) {
        mix=(target-(float)SPEED_VERY_LOW_RPM)/
            (float)(SPEED_LOW_RPM-SPEED_VERY_LOW_RPM);
        kp_percent=(float)SPEED_KP_VERY_LOW_PERCENT+
                   (float)(SPEED_KP_LOW_PERCENT-SPEED_KP_VERY_LOW_PERCENT)*mix;
        ki_percent=(float)SPEED_KI_VERY_LOW_PERCENT+
                   (float)(SPEED_KI_LOW_PERCENT-SPEED_KI_VERY_LOW_PERCENT)*mix;
    } else { *kp=1.0f; *ki=1.0f; return; }
    *kp=kp_percent*0.01f; *ki=ki_percent*0.01f;
}
#endif

static int8_t SpeedOutput(uint8_t wheel,PID_TypeDef *pid,float target,float rpm,
                          float raw_rpm,uint32_t elapsed,float dt,float feedforward,float offset)
{
    float out,limited,old_i,retained,minimum=0.0f;
    int8_t sign=target<0.0f ? -1 : 1;
    uint8_t physical=wheel;
#if DRIVE_PAIRS_SWAPPED
    physical=(uint8_t)(1U-wheel);
#endif
    target=Abs(target);
#if TRACK_LOW_SPEED_ZONE_ENABLE
    {
        float kp_scale,ki_scale;
        SpeedGainScale(target,&kp_scale,&ki_scale);
        pid->Kp=TRACK_SPEED_KP*kp_scale; pid->Ki=TRACK_SPEED_KI*ki_scale;
    }
#endif
    if(target==0.0f) {
        SaveLoad(wheel,pid); PID_Reset(pid);
        still_ms[wheel]=boost_left_ms[wheel]=moving_ms[wheel]=reverse_wait_ms[wheel]=0;
        boost_used[wheel]=0; drive_sign[wheel]=0; output_pwm[wheel]=0.0f;
        boost_ramp_ms[wheel]=0;
        return 0;
    }
    if(drive_sign[wheel]!=sign) {
        uint8_t changing=drive_sign[wheel]!=0 || sign*rpm<-TRACK_REVERSE_COAST_RPM;
        SaveLoad(wheel,pid); PID_Reset(pid);
        load_compensation[wheel]=direction_load[wheel][sign<0 ? 1U : 0U];
        still_ms[wheel]=boost_left_ms[wheel]=moving_ms[wheel]=0;
        boost_used[wheel]=0; drive_sign[wheel]=sign;
        reverse_wait_ms[wheel]=changing ? 1U : 0U; output_pwm[wheel]=0.0f;
        boost_ramp_ms[wheel]=0;
    }
    pid->target=sign*target; pid->actual=rpm; rpm*=sign; raw_rpm*=sign;
    if(reverse_wait_ms[wheel]) {
        reverse_wait_ms[wheel]+=elapsed;
        if(reverse_wait_ms[wheel]<=TRACK_REVERSE_BLANK_MS ||
           (rpm<-TRACK_REVERSE_COAST_RPM && reverse_wait_ms[wheel]<=TRACK_REVERSE_COAST_MAX_MS))
            return 0;
        reverse_wait_ms[wheel]=0;
    }
    if(rpm>=TRACK_BOOST_MOVING_RPM) {
        if(moving_ms[wheel]<TRACK_BOOST_REARM_MS) moving_ms[wheel]+=elapsed;
        still_ms[wheel]=0; boost_left_ms[wheel]=0;
        if(moving_ms[wheel]>=TRACK_BOOST_REARM_MS) boost_used[wheel]=0;
    } else {
        moving_ms[wheel]=0;
        if(raw_rpm>0.0f) { still_ms[wheel]=0; boost_left_ms[wheel]=0; }
        else if(Abs(rpm)<TRACK_BOOST_MOVING_RPM) {
            if(still_ms[wheel]<ENCODER_NO_PULSE_MS) still_ms[wheel]+=elapsed;
            if(!boost_used[wheel] && still_ms[wheel]>=TRACK_BOOST_WAIT_MS) {
                boost_used[wheel]=1; boost_left_ms[wheel]=TRACK_BOOST_DURATION_MS;
                boost_ramp_ms[wheel]=0;
            }
        } else { still_ms[wheel]=0; boost_left_ms[wheel]=0; }
    }
    if(boost_left_ms[wheel]) {
        minimum=TRACK_BOOST_PWM;
        if(boost_left_ms[wheel]<=elapsed) minimum+=TRACK_BOOST_EXTRA_PWM;
    }
    if(sign<0) {
        feedforward=physical==0 ? LEFT_REVERSE_FEEDFORWARD : RIGHT_REVERSE_FEEDFORWARD;
        offset=physical==0 ? LEFT_REVERSE_DRIVE_OFFSET_PWM : RIGHT_REVERSE_DRIVE_OFFSET_PWM;
    }
#if DRIVE_PAIRS_SWAPPED
    else {
        feedforward=physical==0 ? LEFT_SPEED_FEEDFORWARD : RIGHT_SPEED_FEEDFORWARD;
        offset=physical==0 ? LEFT_DRIVE_OFFSET_PWM : RIGHT_DRIVE_OFFSET_PWM;
    }
#endif
    old_i=pid->integral;
    out=PID_Step(pid,target-rpm,dt,offset+load_compensation[wheel]+feedforward*target,
                 minimum,(float)TRACK_MAX_PWM);
    /* 饱和由PID内部处理，外部PWM斜率受限时也冻结同向积分，防止低速起转猛冲。 */
    limited=output_pwm[wheel]+Limit(out-output_pwm[wheel],-TRACK_PWM_FALL_PER_TICK,TRACK_PWM_RISE_PER_TICK);
    if((out>limited && target>rpm) || (out<limited && target<rpm)) pid->integral=old_i;
    if(rpm>target+TRACK_SPEED_COAST_MARGIN) {
        /* 先把已验证的稳态部分等量转移到前馈，再卸瞬态积分；总补偿不增加。
         * 否则正常减速会把高负载轮的有效积分连同瞬态一起清掉，下一段需重新起转。 */
        if(pid->integral>0.0f) {
            retained=Limit(direction_load[wheel][sign<0 ? 1U : 0U]-load_compensation[wheel],0.0f,pid->integral);
            load_compensation[wheel]+=retained; pid->integral-=retained;
        }
        pid->integral*=0.5f; limited=0.0f;
    }
    output_pwm[wheel]=Limit(limited,0.0f,(float)TRACK_MAX_PWM);
    /* 提前缓存有效稳态负载；等命令减到0才保存，会因惯性超速而丢掉整段有效积分。
     * 覆盖本方向估计，不叠加、不改变当前PI，换向仍只继承对应方向的补偿。 */
    if(rpm>=TRACK_LOAD_LEARN_MIN_RPM && Abs(target-rpm)<=TRACK_LOAD_LEARN_ERROR_RPM &&
       !boost_left_ms[wheel] && Abs(out-limited)<=TRACK_LOAD_LEARN_PWM_MARGIN &&
       out<(float)TRACK_MAX_PWM-TRACK_LOAD_LEARN_PWM_MARGIN && pid->integral>0.0f)
        direction_load[wheel][sign<0 ? 1U : 0U]=Limit(load_compensation[wheel]+pid->integral,0.0f,TRACK_LOAD_COMP_MAX_PWM);
    if(boost_left_ms[wheel]) {
        boost_ramp_ms[wheel]+=elapsed;
        /* 计时从真正达到助推PWM开始，避免限斜率把两帧助推全耗在爬升上。 */
        if(output_pwm[wheel]>=minimum || boost_ramp_ms[wheel]>=TRACK_BOOST_RAMP_MAX_MS)
            boost_left_ms[wheel]=boost_left_ms[wheel]>elapsed ? boost_left_ms[wheel]-elapsed : 0;
        if(boost_ramp_ms[wheel]>=TRACK_BOOST_RAMP_MAX_MS+TRACK_BOOST_DURATION_MS) boost_left_ms[wheel]=0;
    }
    return sign*(int8_t)(output_pwm[wheel]+0.5f);
}
static uint8_t EncoderFault(uint8_t wheel,int32_t pulses,float rpm,float command,int8_t previous_pwm,uint32_t elapsed)
{
    uint8_t i; int8_t sign=command<0.0f ? -1 : (command>0.0f ? 1 : 0);
    int32_t positive=0,negative=0,aligned;
    (void)rpm; /* 计数方向窗替代瞬时RPM，零计数也推进时间但不清方向证据。 */
    if(!sign || elapsed>2U*TRACK_PERIOD_MS) {
        no_pulse_ms[wheel]=reverse_ms[wheel]=0; fault_sign[wheel]=0; return 0;
    }
    if(sign!=fault_sign[wheel]) {
        fault_sign[wheel]=sign; no_pulse_ms[wheel]=reverse_ms[wheel]=0;
        direction_grace_ms[wheel]=ENCODER_REVERSAL_GRACE_MS;
        direction_time_ms[wheel]=0; direction_index[wheel]=0;
        for(i=0;i<ENCODER_DIRECTION_WINDOW;++i) direction_window[wheel][i]=0;
    }
    if(direction_grace_ms[wheel]) {
        /* 换向宽容从新方向实际施加PWM开始；撤驱动等待惯性期间不消耗宽容。
         * 仅命令过零但PWM尚未换向时，旧方向脉冲属于正常滑行。 */
        if((int32_t)previous_pwm*sign<=0) return 0;
        direction_grace_ms[wheel]=direction_grace_ms[wheel]>elapsed ? direction_grace_ms[wheel]-elapsed : 0;
        return 0;
    }
    if(pulses) no_pulse_ms[wheel]=0;
    else if(Abs((float)previous_pwm)>=ENCODER_FAULT_MIN_PWM && no_pulse_ms[wheel]<ENCODER_NO_PULSE_MS)
        no_pulse_ms[wheel]+=elapsed; /* 低于门限暂停，不清累计。 */
    aligned=(int32_t)previous_pwm*sign>0 ? sign*pulses : 0;
    direction_window[wheel][direction_index[wheel]]=aligned;
    direction_index[wheel]=(uint8_t)((direction_index[wheel]+1U)%ENCODER_DIRECTION_WINDOW);
    if(direction_time_ms[wheel]<ENCODER_REVERSE_MS) direction_time_ms[wheel]+=elapsed;
    for(i=0;i<ENCODER_DIRECTION_WINDOW;++i) {
        aligned=direction_window[wheel][i];
        if(aligned>0) positive+=aligned; else negative-=aligned;
    }
    reverse_ms[wheel]=direction_time_ms[wheel];
    return no_pulse_ms[wheel]>=ENCODER_NO_PULSE_MS ||
           (direction_time_ms[wheel]>=ENCODER_REVERSE_MS &&
            negative-positive>=(int32_t)ENCODER_DIRECTION_MIN_COUNTS &&
            negative*100>=(positive+negative)*(int32_t)ENCODER_REVERSE_PERCENT);
}
static void FilterSpeed(int32_t left_counts,int32_t right_counts,uint32_t elapsed,float dt)
{
    uint8_t i; int32_t sum_left=0,sum_right=0; uint32_t sum_ms=0;
    float alpha=dt/(TRACK_RPM_FILTER_TAU+dt),left_sample,right_sample,mix;
    rpm_window[0][rpm_index]=left_counts; rpm_window[1][rpm_index]=right_counts;
    rpm_window_ms[rpm_index]=elapsed;
    rpm_index=(uint8_t)((rpm_index+1U)%TRACK_RPM_WINDOW_SAMPLES);
    for(i=0;i<TRACK_RPM_WINDOW_SAMPLES;++i) {
        sum_left+=rpm_window[0][i]; sum_right+=rpm_window[1][i]; sum_ms+=rpm_window_ms[i];
    }
#if DRIVE_PAIRS_SWAPPED
    left_sample=(float)sum_left*60000.0f/(RIGHT_COUNTS_PER_REV*sum_ms);
    right_sample=(float)sum_right*60000.0f/(LEFT_COUNTS_PER_REV*sum_ms);
#else
    left_sample=(float)sum_left*60000.0f/(LEFT_COUNTS_PER_REV*sum_ms);
    right_sample=(float)sum_right*60000.0f/(RIGHT_COUNTS_PER_REV*sum_ms);
#endif
    mix=Limit((TRACK_LOW_SPEED_RPM-Abs(left_command))/(TRACK_LOW_SPEED_RPM-TRACK_LOW_SPEED_FULL_RPM),0.0f,1.0f);
    left_sample=left_rpm+mix*(left_sample-left_rpm);
    mix=Limit((TRACK_LOW_SPEED_RPM-Abs(right_command))/(TRACK_LOW_SPEED_RPM-TRACK_LOW_SPEED_FULL_RPM),0.0f,1.0f);
    right_sample=right_rpm+mix*(right_sample-right_rpm);
    left_filtered+=alpha*(left_sample-left_filtered); right_filtered+=alpha*(right_sample-right_filtered);
}
static float SlewTarget(float command,float target,float rise,float fall)
{
    float step=Abs(target)>Abs(command) ? rise : fall;
    if(command*target<0.0f) {
        if(Abs(command)<=fall) return 0.0f;
        return command+(command<0.0f ? fall : -fall);
    }
    return command+Limit(target-command,-step,step);
}
static void DriveTargets(uint32_t elapsed,float dt)
{
    float rise=(steering_urgent ? TRACK_EDGE_SLEW_RPM_PER_S : TRACK_TARGET_SLEW_RPM_PER_S)*TRACK_PERIOD_MS*0.001f;
    float fall=rise;
    if(!Track_IsRunning()) {
        left_target=right_target=left_command=right_command=0.0f; SetPWM(0,0); return;
    }
    if(state==CAR_CORNER_CONFIRM || state==CAR_CORNER_APPROACH) {
        rise=TRACK_APPROACH_ACCEL_RPM; fall=TRACK_APPROACH_DECEL_RPM;
    } else if(state==CAR_TURN_FAST || state==CAR_TURN_SLOW || state==CAR_SEARCH || state==CAR_SPEED_TEST) {
        rise=TRACK_TURN_ACCEL_RPM; fall=TRACK_TURN_DECEL_RPM;
    } else if(state==CAR_EXIT || state==CAR_ALIGN) { rise=TRACK_EXIT_ACCEL_RPM; fall=TRACK_EXIT_DECEL_RPM; }
    if(elapsed>2U*TRACK_PERIOD_MS) elapsed=TRACK_PERIOD_MS;
#if TRACK_ADAPTIVE_SPEED_ENABLE
    if(state!=CAR_TRACK) AdaptiveSync(motion_mode,left_target,right_target,0,0.0f);
#endif
    /* 每周期限制轮速变化，防止数字探头跳一格时车头猛甩。 */
    left_command=SlewTarget(left_command,left_target,rise,fall);
    right_command=SlewTarget(right_command,right_target,rise,fall);
    SetPWM(SpeedOutput(0,&left_pid,left_command,left_filtered,left_rpm,elapsed,dt,LEFT_SPEED_FEEDFORWARD,LEFT_DRIVE_OFFSET_PWM),
           SpeedOutput(1,&right_pid,right_command,right_filtered,right_rpm,elapsed,dt,RIGHT_SPEED_FEEDFORWARD,RIGHT_DRIVE_OFFSET_PWM));
}
/* 步骤 4：横带与偏侧弯线分开处理。全黑/六路以上强横带才冻结位置纠偏；
 * 四五路弱线索只记忆并减速，仍允许跟随侧边。不能一见宽线就盲目前进。
 */
#if TRACK_ENABLE_CORNERS
static uint8_t BlackSpan(uint8_t mask)
{
    uint8_t first=0,last=0,i;
    if(!mask) return 0;
    while(!(mask&(1U<<first))) ++first;
    for(i=first;i<8;++i) if(mask&(1U<<i)) last=i;
    return (uint8_t)(last-first+1U);
}
static void VoteCornerDirection(int8_t direction)
{
    if(direction) {
        corner_vote+=direction;
        if(corner_vote>CORNER_DIRECTION_VOTE_MAX) corner_vote=CORNER_DIRECTION_VOTE_MAX;
        if(corner_vote<-CORNER_DIRECTION_VOTE_MAX) corner_vote=-CORNER_DIRECTION_VOTE_MAX;
    }
    wide_dir=corner_vote>=CORNER_DIRECTION_VOTES ? 1 :
             (corner_vote<=-CORNER_DIRECTION_VOTES ? -1 : 0);
}
#endif
static uint8_t ObserveWide(uint32_t now,uint32_t elapsed,uint8_t count,float forward_mm)
{
#if TRACK_ENABLE_CORNERS
    uint8_t mask=sensor_mask,span,broad,strong;
#if TRACK_SENSOR_FILTER_ENABLE
    mask=sensor_raw; count=Bits(mask);
#endif
    span=BlackSpan(mask);
    int8_t direction=0;
    if(!corner_armed) { ClearWide(); return 0; }
    broad=count>=CORNER_WIDTH_MIN_PROBES && span>=normal_line_width+CORNER_WIDTH_EXTRA_PROBES &&
          Contiguous(mask) && (mask&0x81U) && (mask&0x18U);
    if(mask==0xFFU) broad=1;
    if(wide_active && forward_mm>0.0f) wide_travel_mm+=forward_mm;
    if(broad) {
        edge_ms=0; edge_dir=0;
        if(!wide_active) { ClearWide(); wide_active=1; wide_started=now; }
        strong=(mask&0x81U)==0x81U || span>=normal_line_width+CORNER_STRONG_WIDTH_EXTRA;
#if TRACK_SENSOR_FILTER_ENABLE
        if(Bits(sensor_filtered)>=CORNER_WIDTH_MIN_PROBES &&
           Contiguous(sensor_filtered) && (sensor_filtered&0x81U) && (sensor_filtered&0x18U))
#endif
        corner_confidence+=(strong ? CORNER_CONFIDENCE_STRONG : CORNER_CONFIDENCE_WEAK);
        if(corner_confidence>CORNER_CONFIDENCE_MAX) corner_confidence=CORNER_CONFIDENCE_MAX;
        wide_ms+=elapsed; wide_gap_ms=0; wide_last_seen=now; wide_last_mm=wide_travel_mm;
        if(count>wide_peak_count) wide_peak_count=count;
        wide_center_ms=wide_tail_ms=0; wide_center_mm=0.0f;
        if((mask&0x01U) && !(mask&0x80U)) direction=-1;
        else if((mask&0x80U) && !(mask&0x01U)) direction=1;
        VoteCornerDirection(direction);
        if(corner_confidence>=CORNER_CONFIDENCE_REQUIRED && wide_ms>=CORNER_CONFIRM_MS)
            wide_strong=1; /* 置信度确认，而非单帧黑数决定是否推进。 */
        state=CAR_CORNER_CONFIRM; lost_ms=0;
        return wide_strong;
    }
    if(wide_active && !wide_strong) {
        wide_gap_ms+=elapsed;
        /* 窄线恢复时衰减；短白缝在原40ms窗口内保留候选，不增加分数。 */
        if(count && corner_confidence) corner_confidence-=CORNER_CONFIDENCE_DECAY;
        if(wide_gap_ms>CORNER_CONFIRM_GAP_MS) ClearWide();
        return 0;
    }
    if(wide_active && count && Contiguous(mask)) {
        if((mask&0x01U) && !(mask&0x80U)) direction=-1;
        else if((mask&0x80U) && !(mask&0x01U)) direction=1;
        if(direction && (!wide_dir || direction==wide_dir)) {
            VoteCornerDirection(direction); wide_last_seen=now; wide_last_mm=wide_travel_mm;
        }
    }
    if(wide_active && (uint32_t)(now-wide_last_seen)>CORNER_MEMORY_MS) ClearWide();
#else
    (void)now; (void)elapsed; (void)count; (void)forward_mm;
#endif
    return 0;
}
#if TRACK_NONLINEAR_FOLLOW_ENABLE
/* Integrated slopes give continuous, odd P correction with no centre dead zone. */
static float NonlinearError(float error)
{
    float x=Abs(error),value;
    if(x<=TRACK_CENTER_ZONE) value=x*(float)TRACK_CENTER_GAIN_PERCENT*0.01f;
    else if(x<=TRACK_MEDIUM_ZONE)
        value=(float)TRACK_CENTER_ZONE*(float)TRACK_CENTER_GAIN_PERCENT*0.01f+
              (x-TRACK_CENTER_ZONE)*(float)TRACK_MEDIUM_GAIN_PERCENT*0.01f;
    else value=(float)TRACK_CENTER_ZONE*(float)TRACK_CENTER_GAIN_PERCENT*0.01f+
               (float)(TRACK_MEDIUM_ZONE-TRACK_CENTER_ZONE)*(float)TRACK_MEDIUM_GAIN_PERCENT*0.01f+
               (x-TRACK_MEDIUM_ZONE)*(float)TRACK_OUTER_GAIN_PERCENT*0.01f;
    return error<0.0f ? -value : value;
}
#endif
/* 普通弯道：边缘探头已看到线时立即减速并朝该侧纠偏。
 * 中间位置保留原滤波；边缘采用较快滤波与轮速变化，避免等全白才转。
 */
static void FollowLine(uint32_t now,float error,float dt,float speed_cap)
{
    float magnitude,base,correction,tau=TRACK_LINE_FILTER_TAU;
    float ratio=TRACK_MAX_CORRECTION_RATIO;
    float control_error=error,position_bias=0.0f;
    last_error=error;
    steering_urgent=Abs(error)>=TRACK_EDGE_ERROR;
    if(steering_urgent) { tau=TRACK_EDGE_FILTER_TAU; ratio=TRACK_EDGE_CORRECTION_RATIO; }
    line_seen=now;
    if(Abs(error)>=TRACK_HINT_ERROR && Bits(sensor_mask)<=normal_line_width+1U && Contiguous(sensor_mask)) {
        turn_hint=error<0.0f ? -1 : 1; hint_seen=now;
    }
    if(Abs(error)<=TRACK_CENTER_ERROR) {
        control_error=Abs(error)>TRACK_CENTER_DEADBAND ?
                      (error<0.0f ? -1.0f : 1.0f)*(Abs(error)-TRACK_CENTER_DEADBAND) : 0.0f;
    }
    error_filtered+=dt/(tau+dt)*(control_error-error_filtered);
#if TRACK_NONLINEAR_FOLLOW_ENABLE
    nonlinear_error_filtered+=dt/(tau+dt)*(error-nonlinear_error_filtered);
#endif
    magnitude=Abs(error_filtered);
    if(steering_urgent && Abs(error)>magnitude) magnitude=Abs(error);
    base=TRACK_BASE_RPM-TRACK_BEND_SLOWDOWN*magnitude;
    base=Limit(base,speed_cap<TRACK_BEND_MIN_RPM ? speed_cap : TRACK_BEND_MIN_RPM,speed_cap);
#if TRACK_ADAPTIVE_SPEED_ENABLE
    if(state==CAR_TRACK) base=AdaptiveStep(error,error_rate,speed_cap);
    else AdaptiveSync(TRACK_MOTION_FORWARD,0.0f,0.0f,1,base);
#endif
#if TRACK_NONLINEAR_FOLLOW_ENABLE
    position_bias=line_pid.Kp*(NonlinearError(nonlinear_error_filtered)-error_filtered);
#endif
#if TRACK_ERROR_PREDICT_ENABLE
    /* P-only lookahead; ResetPID disables original Kd in this configuration.
     * The rate term replaces D, so neither two first derivatives nor a predicted
     * error differentiated again are added. This experiment stays default OFF. */
    position_bias+=line_pid.Kp*(float)TRACK_ERROR_PREDICT_MS*0.001f*error_rate;
#endif
    correction=PID_Step(&line_pid,error_filtered,dt,position_bias,-base*ratio,base*ratio);
    if(steering_urgent) {
        /* 滤波尚未追上新的边缘位置时，也不能给出反方向或近似直行命令。 */
        if(error<0.0f && correction>-base*TRACK_EDGE_MIN_CORRECTION) correction=-base*TRACK_EDGE_MIN_CORRECTION;
        if(error>0.0f && correction< base*TRACK_EDGE_MIN_CORRECTION) correction= base*TRACK_EDGE_MIN_CORRECTION;
    }
#if TRACK_ADAPTIVE_SPEED_ENABLE
    if(state==CAR_TRACK) {
        float maximum=base>(float)TRACK_MIN_CONTINUOUS_RPM ? base-(float)TRACK_MIN_CONTINUOUS_RPM : 0.0f;
        correction=Limit(correction,-maximum,maximum);
    }
#endif
    motion_mode=TRACK_MOTION_FORWARD;
    left_target=base+correction; right_target=base-correction;
}
/* 可靠角点 → 有效hint → 已确认且未过期的历史；弱线索只许小角试探。 */
static int8_t SelectDirection(uint32_t now,int8_t observed)
{
    direction_weak=0;
    if(observed) return observed;
    if(turn_hint && (uint32_t)(now-hint_seen)<=TRACK_TURN_HINT_MS) return turn_hint;
    if(last_turn_dir && (uint32_t)(now-last_turn_seen)<=TRACK_TURN_HISTORY_MS) return last_turn_dir;
    direction_weak=1;
    if((uint32_t)(now-line_seen)<=TRACK_RECENT_LINE_MS && Abs(last_error)>TRACK_CENTER_DEADBAND)
        return last_error<0.0f ? -1 : 1;
    return 0; /* 完全对称且无证据时，不任意选择左/右。 */
}
static void SetTargets(TrackMotionMode mode,float left,float right)
{
    if(!Track_IsRunning()) return;
    if((mode==TRACK_MOTION_FORWARD && (left<0.0f || right<0.0f)) ||
       (mode==TRACK_MOTION_PIVOT && (left<0.0f || right<0.0f || (left!=0.0f && right!=0.0f))) ||
       (mode==TRACK_MOTION_SPIN && left*right>0.0f)) { HaltReason(TRACK_STOP_MODE); return; }
    motion_mode=mode; left_target=left; right_target=right;
#if TRACK_ADAPTIVE_SPEED_ENABLE
    AdaptiveSync(mode,left,right,0,0.0f);
#endif
}
static void RecoveryBegin(uint32_t now)
{
    if(recovery_active) return;
    recovery_active=1; recovery_start_ms=now; recovery_attempts=0;
    recovery_total_angle=recovery_distance=recovery_stable_mm=0.0f;
    recovery_stable_ms=0;
}
static uint8_t RecoveryStep(uint32_t now,uint32_t elapsed,uint8_t count,float error,
                            float left_mm,float right_mm,float yaw)
{
    float forward_mm=(left_mm+right_mm)*0.5f;
    if(!recovery_active) return 0;
    recovery_total_angle+=Abs(yaw);
    recovery_distance+=(Abs(left_mm)+Abs(right_mm))*0.5f;
    if((uint32_t)(now-recovery_start_ms)>=TRACK_RECOVERY_MAX_MS) { HaltReason(TRACK_STOP_RECOVERY_TIME); return 1; }
    if(recovery_total_angle>=TRACK_RECOVERY_MAX_ANGLE_RAD) { HaltReason(TRACK_STOP_RECOVERY_ANGLE); return 1; }
    if(recovery_distance>=TRACK_RECOVERY_MAX_DISTANCE_MM) { HaltReason(TRACK_STOP_RECOVERY_DISTANCE); return 1; }
    if(state==CAR_TRACK && !wide_active && count && count<=normal_line_width+1U &&
       Contiguous(sensor_mask) && Abs(error)<=TRACK_RECOVERY_STABLE_ERROR && forward_mm>0.0f) {
        recovery_stable_ms+=elapsed; recovery_stable_mm+=forward_mm;
        if(recovery_stable_ms>=TRACK_RECOVERY_STABLE_MS && recovery_stable_mm>=TRACK_RECOVERY_STABLE_MM) {
            recovery_active=0; recovery_attempts=0; recovery_start_ms=recovery_stable_ms=0;
            recovery_total_angle=recovery_distance=recovery_stable_mm=0.0f;
        }
    } else { recovery_stable_ms=0; recovery_stable_mm=0.0f; }
    return 0;
}
static void BeginApproach(uint32_t now)
{
    float half_width=wide_last_mm*0.5f,after_wide=wide_travel_mm-wide_last_mm;
    if(half_width<CORNER_WIDE_HALF_MIN_MM) half_width=LINE_HALF_WIDTH_MM;
    half_width=Limit(half_width,CORNER_WIDE_HALF_MIN_MM,CORNER_WIDE_HALF_MAX_MM);
    approach_goal_mm=SENSOR_FRONT_OFFSET_MM-half_width-after_wide+CORNER_ADVANCE_TRIM_MM;
    approach_goal_mm=Limit(approach_goal_mm,CORNER_APPROACH_MIN_MM,CORNER_APPROACH_TARGET_MAX_MM);
    RecoveryBegin(now-lost_ms+TRACK_PERIOD_MS);
    corner_dir=SelectDirection(now,wide_dir);
    ClearWide(); corner_armed=0; centered_ms=0; rearm_mm=0.0f;
    side_candidate=0; side_ms=0; edge_ms=0; edge_dir=0;
    approach_mm=approach_yaw=0.0f; phase_started=now; state=CAR_CORNER_APPROACH;
    approach_sync_ms=0; approach_synced=0;
    /* 入态首帧也使用同步段低速，不能先请求50RPM再在下一帧退回28。 */
    PID_Reset(&line_pid); SetTargets(TRACK_MOTION_FORWARD,CORNER_APPROACH_END_RPM,CORNER_APPROACH_END_RPM);
}
static void BeginScan(uint32_t now,int8_t direction,uint8_t is_corner)
{
    RecoveryBegin(now);
    if(recovery_attempts>=TRACK_RECOVERY_MAX_ATTEMPTS) { HaltReason(TRACK_STOP_RECOVERY_ATTEMPTS); return; }
    ++recovery_attempts;
    /* 局部扫描可以重新计角度；速度命令、速度积分、故障证据不重置。 */
    ClearWide(); PID_Reset(&line_pid);
    state=is_corner==1 && !direction_weak && direction ? CAR_TURN_FAST : CAR_SEARCH;
    edge_align_active=is_corner==2; search_near_exit=is_corner==3;
    if(is_corner!=3) { alignment_from_corner=is_corner==1; exit_dir=direction; }
    corner_dir=direction;
    phase_started=now; turn_angle=0.0f; sweep_number=1; search_stage=0;
    center_samples=acquire_valid=0; lost_ms=0;
    side_candidate=0; side_ms=0; edge_ms=0; edge_dir=0;
}
static void BeginEdgeAlign(uint32_t now,int8_t direction)
{
    direction_weak=0; BeginScan(now,direction,2);
    SetTargets(TRACK_MOTION_PIVOT,direction>0 ? TRACK_EDGE_PIVOT_RPM : 0.0f,
                                    direction<0 ? TRACK_EDGE_PIVOT_RPM : 0.0f);
}
static uint8_t AcquireLine(uint8_t count,float error,int8_t direction,float progress,float min_angle)
{
    uint8_t valid=count && count<=normal_line_width+1U && Contiguous(sensor_mask) &&
                  (sensor_mask&0x18U) && !(sensor_mask&0x81U) && Abs(error)<=TRACK_ACQUIRE_ERROR;
    uint8_t trend=!acquire_valid || direction*(error-acquire_error)<=TRACK_ACQUIRE_TREND_TOL;
    if((min_angle>0.0f && progress<min_angle) || !valid || !trend || direction*error<-TRACK_ACQUIRE_CROSS_ERROR) {
        center_samples=0;
    } else if(center_samples<CORNER_CENTER_SAMPLES) ++center_samples;
    acquire_error=error; acquire_valid=count && count<=normal_line_width+1U && Contiguous(sensor_mask);
    return center_samples>=CORNER_CENTER_SAMPLES;
}
static void BeginAlign(uint32_t now,float error)
{
#if TRACK_ALIGN_TREND_ENABLE
    AlignEnhanceReset(error);
#endif
    state=CAR_ALIGN; phase_started=now; align_angle=turn_angle;
    align_travel_mm=0.0f; align_center_ms=lost_ms=0;
    if(!search_near_exit) exit_dir=corner_dir;
    edge_align_active=0;
    PID_Reset(&line_pid); last_error=error_filtered=error;
#if TRACK_NONLINEAR_FOLLOW_ENABLE
    nonlinear_error_filtered=error;
#endif
}
static void Approach(uint32_t now,uint32_t elapsed,float dt,float forward_mm,float yaw)
{
    float correction,base=CORNER_APPROACH_RPM;
    approach_mm+=forward_mm; approach_yaw+=yaw;
    if(approach_mm>CORNER_APPROACH_MAX_MM || Abs(approach_yaw)>CORNER_APPROACH_MAX_YAW_RAD ||
       (uint32_t)(now-phase_started)>=CORNER_APPROACH_MAX_MS) { Halt(CAR_STOP); return; }
    if(approach_mm>=CORNER_APPROACH_MIN_MM && approach_mm>=approach_goal_mm) {
        BeginScan(now,corner_dir,1); /* 命令连续，随后自然从前进过零进入旋转。 */
        SetTargets(TRACK_MOTION_SPIN,corner_dir*CORNER_RPM,-corner_dir*CORNER_RPM);
    } else {
        if(!approach_synced) {
            if(Abs(right_filtered-left_filtered)<=CORNER_APPROACH_SYNC_RPM) approach_sync_ms+=elapsed;
            else approach_sync_ms=0;
            if(approach_sync_ms>=CORNER_APPROACH_SYNC_MS) approach_synced=1;
        }
        if(!approach_synced || approach_goal_mm-approach_mm<=CORNER_APPROACH_SLOW_MM) base=CORNER_APPROACH_END_RPM;
        correction=Limit((right_filtered-left_filtered)*CORNER_APPROACH_SYNC_KP+approach_yaw*CORNER_APPROACH_YAW_KP,
                         -CORNER_APPROACH_MAX_DIFF_RPM*0.5f,CORNER_APPROACH_MAX_DIFF_RPM*0.5f);
        SetTargets(TRACK_MOTION_FORWARD,base+correction,base-correction);
    }
    DriveTargets(elapsed,dt);
}
#if TRACK_TURN_CONTINUITY_ENABLE
/* progress radians, requested RPM. Existing angle limits and acquisition stay. */
static float TurnRequested(float progress)
{
    float width=(float)TRACK_TURN_BLEND_DEG*0.01745329252f; /* radians */
    if(progress<=CORNER_FAST_END_RAD-width) return CORNER_RPM;
    if(progress<CORNER_FAST_END_RAD)
        return CORNER_RPM+(CORNER_MID_RPM-CORNER_RPM)*
               ((progress-(CORNER_FAST_END_RAD-width))/width);
    if(progress<=CORNER_SLOW_END_RAD-width) return CORNER_MID_RPM;
    if(progress<CORNER_SLOW_END_RAD)
        return CORNER_MID_RPM+(CORNER_FIND_RPM-CORNER_MID_RPM)*
               ((progress-(CORNER_SLOW_END_RAD-width))/width);
    return CORNER_FIND_RPM;
}
#endif
static void TurnCorner(uint32_t now,uint32_t elapsed,float dt,uint8_t count,float error)
{
    float progress=-corner_dir*turn_angle,rpm=CORNER_RPM;
    if(progress<-TRACK_WRONG_TURN_RAD) { Halt(CAR_ENCODER_FAULT); return; }
    if((uint32_t)(now-phase_started)>=CORNER_TURN_MAX_MS) { Halt(CAR_STOP); return; }
    if(progress>=CORNER_FAST_END_RAD) { state=CAR_TURN_SLOW; rpm=CORNER_MID_RPM; }
    if(progress>=CORNER_SLOW_END_RAD) rpm=CORNER_FIND_RPM;
#if TRACK_TURN_CONTINUITY_ENABLE
    rpm=TurnRequested(progress);
#endif
    if(AcquireLine(count,error,corner_dir,progress,CORNER_MIN_ANGLE_RAD)) {
        BeginAlign(now,error);
        SetTargets(TRACK_MOTION_FORWARD,TRACK_ALIGN_BASE_RPM,TRACK_ALIGN_BASE_RPM);
        DriveTargets(elapsed,dt); return;
    }
    if(progress>=CORNER_SCAN_LIMIT_RAD) {
        exit_dir=corner_dir; direction_weak=0;
        BeginScan(now,-corner_dir,3); /* 105°处只在附近反向15°，不从头扫另一侧。 */
    }
    SetTargets(TRACK_MOTION_SPIN,corner_dir*rpm,-corner_dir*rpm); DriveTargets(elapsed,dt);
}
static void Scan(uint32_t now,uint32_t elapsed,float dt,uint8_t count,float error)
{
    float progress=-corner_dir*turn_angle,rpm=search_stage==0 ? TRACK_SEARCH_SMALL_RPM : TRACK_SEARCH_RPM;
    float limit=search_near_exit ? TRACK_NEAR_LINE_SCAN_RAD : TRACK_SEARCH_SCAN_RAD;
    if(state!=CAR_SEARCH) return;
    if(direction_weak) limit=TRACK_UNKNOWN_PROBE_RAD;
    if(direction_weak || search_near_exit) rpm=CORNER_FIND_RPM;
    if((uint32_t)(now-phase_started)>=TRACK_SEARCH_MAX_MS) { Halt(CAR_STOP); return; }
    if(!corner_dir) {
        /* 无任何侧向线索，只保守停车观察短时间，不猜固定左/右。 */
        if(!alignment_from_corner && AcquireLine(count,error,0,0.0f,0.0f)) {
            BeginAlign(now,error); SetTargets(TRACK_MOTION_FORWARD,TRACK_ALIGN_BASE_RPM,TRACK_ALIGN_BASE_RPM);
            DriveTargets(elapsed,dt); return;
        }
        if(count && Abs(error)>=TRACK_HINT_ERROR && Contiguous(sensor_mask)) {
            corner_dir=error<0.0f ? -1 : 1; direction_weak=1;
        } else {
            SetTargets(TRACK_MOTION_FORWARD,0.0f,0.0f); DriveTargets(elapsed,dt);
            if((uint32_t)(now-phase_started)>=TRACK_UNKNOWN_PROBE_MS) HaltReason(TRACK_STOP_NO_DIRECTION);
            return;
        }
    }
    if(AcquireLine(count,error,corner_dir,progress,alignment_from_corner && direction_weak ? CORNER_MIN_ANGLE_RAD : 0.0f)) {
        BeginAlign(now,error); SetTargets(TRACK_MOTION_FORWARD,TRACK_ALIGN_BASE_RPM,TRACK_ALIGN_BASE_RPM);
        DriveTargets(elapsed,dt); return;
    }
    if(edge_align_active) {
        if(progress>=TRACK_EDGE_ALIGN_LIMIT_RAD || (uint32_t)(now-phase_started)>=TRACK_EDGE_ALIGN_MAX_MS) {
            Halt(CAR_STOP); return;
        }
        rpm=count && Abs(error)<=TRACK_HINT_ERROR ? CORNER_ALIGN_RPM : TRACK_EDGE_PIVOT_RPM;
        SetTargets(TRACK_MOTION_PIVOT,corner_dir>0 ? rpm : 0.0f,corner_dir<0 ? rpm : 0.0f);
    } else {
        if(!search_near_exit && !direction_weak && search_stage==0 && progress>=TRACK_SEARCH_SMALL_RAD)
            search_stage=1;
        if(progress>=limit) {
            if(sweep_number>=2 || search_near_exit) { Halt(CAR_STOP); return; }
            ++sweep_number; search_stage=2; corner_dir=-corner_dir; center_samples=acquire_valid=0;
        }
        SetTargets(TRACK_MOTION_SPIN,corner_dir*rpm,-corner_dir*rpm);
    }
    DriveTargets(elapsed,dt);
}
static void Align(uint32_t now,uint32_t elapsed,float dt,uint8_t count,float error,float forward_mm)
{
    float correction,base=TRACK_ALIGN_BASE_RPM,gain=TRACK_ALIGN_KP;
    if(forward_mm>0.0f) align_travel_mm+=forward_mm;
    if((uint32_t)(now-phase_started)>=TRACK_ALIGN_MAX_MS || align_travel_mm>TRACK_ALIGN_MAX_TRAVEL_MM ||
       Abs(turn_angle-align_angle)>TRACK_ALIGN_MAX_YAW_RAD) { Halt(CAR_STOP); return; }
    if(!count) {
        lost_ms+=elapsed; align_center_ms=0;
#if TRACK_ALIGN_TREND_ENABLE
        AlignEnhanceReset(error);
#endif
        if(lost_ms>=TRACK_EXIT_WHITE_GRACE_MS) {
            direction_weak=0; BeginScan(now,exit_dir,3);
            return;
        }
        correction=exit_dir*TRACK_EXIT_PREDICT_DIFF_RPM*0.5f;
        SetTargets(TRACK_MOTION_FORWARD,TRACK_EXIT_PREDICT_RPM+correction,TRACK_EXIT_PREDICT_RPM-correction);
    } else {
        lost_ms=0; last_error=error;
        if(Centered(count,sensor_raw_error) && count<=normal_line_width+1U) align_center_ms+=elapsed; else align_center_ms=0;
#if TRACK_ALIGN_TREND_ENABLE
        AlignEnhance(error,&base,&gain);
#endif
        correction=Limit(error*gain,-TRACK_ALIGN_MAX_DIFF_RPM*0.5f,TRACK_ALIGN_MAX_DIFF_RPM*0.5f);
        SetTargets(TRACK_MOTION_FORWARD,base+correction,base-correction);
        if(align_center_ms>=TRACK_ALIGN_CENTER_MS) {
            state=CAR_EXIT; phase_started=now; exit_center_ms=0; exit_travel_mm=0.0f;
            PID_Reset(&line_pid); error_filtered=error;
#if TRACK_NONLINEAR_FOLLOW_ENABLE
            nonlinear_error_filtered=error;
#endif
            if(alignment_from_corner) { last_turn_dir=exit_dir; last_turn_seen=now; }
        }
    }
    DriveTargets(elapsed,dt);
}
static void ExitLine(uint32_t now,uint32_t elapsed,float dt,uint8_t count,float error,float forward_mm)
{
    float cap=TRACK_EXIT_START_RPM,correction;
    uint32_t age=now-phase_started;
    if(forward_mm>0.0f) exit_travel_mm+=forward_mm;
    if(age>=TRACK_EXIT_MAX_MS) { Halt(CAR_STOP); return; }
    if(!count) {
        lost_ms+=elapsed; exit_center_ms=0;
        if(lost_ms>=TRACK_EXIT_WHITE_GRACE_MS) {
            direction_weak=0; BeginScan(now,exit_dir,3); return;
        }
        correction=exit_dir*TRACK_EXIT_PREDICT_DIFF_RPM*0.5f;
        SetTargets(TRACK_MOTION_FORWARD,TRACK_EXIT_PREDICT_RPM+correction,TRACK_EXIT_PREDICT_RPM-correction);
    } else {
        lost_ms=0;
        if(count<=normal_line_width+1U &&
           (alignment_from_corner ? (Contiguous(sensor_mask) && (sensor_mask&0x18U) &&
                                     !(sensor_mask&0x81U) && Abs(sensor_raw_error)<=TRACK_EXIT_CENTER_ERROR) :
            (Contiguous(sensor_mask) && Abs(sensor_raw_error)<=TRACK_RECOVERY_EXIT_ERROR))) exit_center_ms+=elapsed;
        else exit_center_ms=0;
        /* 时间和线稳定性共同推进速度档；回线还在摆动时保持低速。 */
        if(age>=TRACK_EXIT_MID_MS && exit_center_ms>=TRACK_EXIT_MID_CENTER_MS) cap=TRACK_EXIT_MID_RPM;
        if(age>=TRACK_EXIT_FULL_MS && exit_center_ms>=TRACK_EXIT_CENTER_MS) cap=TRACK_BASE_RPM;
        FollowLine(now,error,dt,cap);
        if(age>=TRACK_EXIT_MIN_MS && exit_center_ms>=TRACK_EXIT_CENTER_MS+TRACK_EXIT_CENTER_MS &&
           exit_travel_mm>=TRACK_EXIT_MIN_TRAVEL_MM) {
            state=CAR_TRACK; /* EXIT已完成速度恢复，不再额外把60RPM降回旧42RPM。 */
        }
    }
    DriveTargets(elapsed,dt);
}
static void LearnAndRearm(uint32_t elapsed,uint8_t count,float error,float forward_mm)
{
    if(!corner_armed) {
        if(forward_mm>0.0f) rearm_mm+=forward_mm;
        if(count && count<=normal_line_width+1U && Contiguous(sensor_mask) && Abs(error)<=TRACK_EDGE_ERROR)
            centered_ms+=elapsed; else centered_ms=0;
        if(centered_ms>=CORNER_REARM_CENTER_MS && rearm_mm>=CORNER_REARM_TRAVEL_MM) corner_armed=1;
    }
    if(!wide_active && state==CAR_TRACK && Centered(count,error) && count>normal_line_width && count<=normal_line_width+1U) {
        if(width_candidate!=count) { width_candidate=count; width_learn_ms=0; }
        width_learn_ms+=elapsed;
        if(width_learn_ms>=TRACK_WIDTH_LEARN_MS) { normal_line_width=count; width_learn_ms=0; }
    } else { width_candidate=0; width_learn_ms=0; }
}
static void TrackLine(uint32_t now,uint32_t elapsed,float dt,uint8_t count,float error,float forward_mm)
{
    float largest,scale; uint8_t hold_wide;
    ResolveSide(elapsed,&error); observed_error=error;
    LearnAndRearm(elapsed,count,sensor_raw_error,forward_mm);
    hold_wide=ObserveWide(now,elapsed,count,forward_mm);
    if(wide_active && ((uint32_t)(now-wide_started)>=CORNER_CONFIRM_MAX_MS || wide_travel_mm>=CORNER_CONFIRM_MAX_TRAVEL_MM)) {
        Halt(CAR_STOP); return;
    }
    if(hold_wide) {
        PID_Reset(&line_pid); SetTargets(TRACK_MOTION_FORWARD,CORNER_APPROACH_RPM,CORNER_APPROACH_RPM);
        DriveTargets(elapsed,dt); return;
    }
    if(line_ambiguous) {
        ambiguous_ms+=elapsed;
        SetTargets(TRACK_MOTION_FORWARD,TRACK_ALIGN_BASE_RPM,TRACK_ALIGN_BASE_RPM);
        if(ambiguous_ms>=TRACK_AMBIGUOUS_MAX_MS) HaltReason(TRACK_STOP_AMBIGUOUS); else DriveTargets(elapsed,dt);
        return;
    }
    ambiguous_ms=0;
    if(count) {
        lost_ms=0;
        if(wide_active) {
            if(Centered(count,sensor_raw_error) && count<=normal_line_width) {
                wide_center_ms+=elapsed;
                if(forward_mm>0.0f) wide_center_mm+=forward_mm;
                if(wide_center_ms>=CORNER_CLEAR_CENTER_MS && wide_center_mm>=CORNER_CLEAR_TRAVEL_MM) ClearWide();
            } else { wide_center_ms=0; wide_center_mm=0.0f; }
            if(!wide_strong && count<=normal_line_width+1U && Abs(error)>=TRACK_ACQUIRE_ERROR) {
                wide_tail_ms+=elapsed;
                if(wide_tail_ms>=TRACK_SIDE_CONFIRM_MS) ClearWide();
            } else wide_tail_ms=0;
            if(wide_active && wide_strong && !wide_center_ms) {
                state=CAR_CORNER_CONFIRM;
                SetTargets(TRACK_MOTION_FORWARD,CORNER_APPROACH_RPM,CORNER_APPROACH_RPM);
                DriveTargets(elapsed,dt); return;
            }
        }
    } else {
        lost_ms+=elapsed;
        if(wide_active && wide_strong && lost_ms>=CORNER_LOST_CONFIRM_MS) {
            BeginApproach(now); DriveTargets(elapsed,dt); return;
        }
        if(lost_ms<TRACK_LOST_GRACE_MS) {
            /* 连续短白缝也记录会话；不能靠1帧中心不断刷新预测预算。 */
            RecoveryBegin(now);
            if(!wide_strong && (uint32_t)(now-recovery_start_ms)>=TRACK_RECOVERY_PREDICT_MAX_MS) {
                BeginScan(now,SelectDirection(now,0),0); Scan(now,elapsed,dt,count,sensor_raw_error); return;
            }
            largest=Abs(left_target)>Abs(right_target) ? Abs(left_target) : Abs(right_target);
            scale=largest>TRACK_BEND_MIN_RPM ? TRACK_BEND_MIN_RPM/largest : 1.0f;
            left_target*=scale; right_target*=scale;
#if TRACK_ADAPTIVE_SPEED_ENABLE
            AdaptiveSync(motion_mode,left_target,right_target,0,0.0f);
#endif
            DriveTargets(elapsed,dt); return;
        }
        RecoveryBegin(now-lost_ms+TRACK_PERIOD_MS);
        BeginScan(now,SelectDirection(now,0),0);
        if(state==CAR_SEARCH) Scan(now,elapsed,dt,count,sensor_raw_error);
        return;
    }
    state=wide_active ? CAR_CORNER_CONFIRM : CAR_TRACK;
    if(!wide_active && count<=normal_line_width+1U && Contiguous(sensor_mask) && Abs(error)>=TRACK_EDGE_ALIGN_ERROR) {
        int8_t direction=error<0.0f ? -1 : 1;
        if(edge_dir!=direction) { edge_dir=direction; edge_ms=0; edge_best_error=Abs(error); }
        if(Abs(error)<edge_best_error-TRACK_CENTER_DEADBAND) { edge_ms=0; edge_best_error=Abs(error); }
        edge_ms+=elapsed;
        if(edge_ms>=TRACK_EDGE_ALIGN_MS) { BeginEdgeAlign(now,direction); DriveTargets(elapsed,dt); return; }
    } else { edge_ms=0; edge_dir=0; }
    if(sensor_mask==0xFFU) {
        PID_Reset(&line_pid); SetTargets(TRACK_MOTION_FORWARD,TRACK_BEND_MIN_RPM,TRACK_BEND_MIN_RPM);
    } else FollowLine(now,error,dt,wide_active || side_candidate ? CORNER_APPROACH_RPM : TRACK_BASE_RPM);
    DriveTargets(elapsed,dt);
}
static int32_t AddTotal(int32_t total,int32_t counts)
{
    if(counts>0 && total>2147483647-counts) return 2147483647;
    if(counts<0 && total<(-2147483647-1)-counts) return (-2147483647-1);
    return total+counts;
}
/* 一个20ms周期只采样一次，然后分派小状态函数；中断不运行此逻辑。 */
static void Control(uint32_t now,uint32_t elapsed)
{
    float error,dt,left_mm,right_mm,forward_mm,yaw;
    int16_t raw_left,raw_right;
    int32_t left_counts,right_counts;
    uint8_t count;
    Encoder_GetPair(&raw_left,&raw_right);
#if DRIVE_PAIRS_SWAPPED
    left_counts=RIGHT_ENCODER_SIGN*(int32_t)raw_right; right_counts=LEFT_ENCODER_SIGN*(int32_t)raw_left;
    left_mm=(float)left_counts*3.14159265f*WHEEL_DIAMETER_MM/RIGHT_COUNTS_PER_REV;
    right_mm=(float)right_counts*3.14159265f*WHEEL_DIAMETER_MM/LEFT_COUNTS_PER_REV;
    left_rpm=(float)left_counts*60000.0f/(RIGHT_COUNTS_PER_REV*elapsed);
    right_rpm=(float)right_counts*60000.0f/(LEFT_COUNTS_PER_REV*elapsed);
#else
    left_counts=LEFT_ENCODER_SIGN*(int32_t)raw_left; right_counts=RIGHT_ENCODER_SIGN*(int32_t)raw_right;
    left_mm=(float)left_counts*3.14159265f*WHEEL_DIAMETER_MM/LEFT_COUNTS_PER_REV;
    right_mm=(float)right_counts*3.14159265f*WHEEL_DIAMETER_MM/RIGHT_COUNTS_PER_REV;
    left_rpm=(float)left_counts*60000.0f/(LEFT_COUNTS_PER_REV*elapsed);
    right_rpm=(float)right_counts*60000.0f/(RIGHT_COUNTS_PER_REV*elapsed);
#endif
    left_total_counts=AddTotal(left_total_counts,left_counts); right_total_counts=AddTotal(right_total_counts,right_counts);
    forward_mm=(left_mm+right_mm)*0.5f; yaw=(right_mm-left_mm)/AXLE_TRACK_MM;
    count=ReadLine(&error); steering_urgent=0;
    if(state==CAR_TURN_FAST || state==CAR_TURN_SLOW || state==CAR_SEARCH)
        observed_error=sensor_raw_error; /* Match raw acquisition/control provenance. */
    if(!Track_IsRunning()) { SetPWM(0,0); return; }
    if(elapsed>TRACK_CONTROL_MAX_GAP_MS) { Halt(CAR_CONTROL_FAULT); return; }
#if TRACK_ERROR_TREND_ENABLE || TRACK_ADAPTIVE_SPEED_ENABLE || TRACK_ERROR_PREDICT_ENABLE
    if(count && (state==CAR_TRACK || state==CAR_ALIGN || state==CAR_EXIT))
        TrendPush(error,(float)elapsed*0.001f);
    else TrendReset(error);
#endif
    if(turn_hint && (uint32_t)(now-hint_seen)>TRACK_TURN_HINT_MS) turn_hint=0;
    if(EncoderFault(0,left_counts,left_rpm,state==CAR_FORWARD ? TRACK_BASE_RPM : left_command,left_pwm,elapsed) ||
       EncoderFault(1,right_counts,right_rpm,state==CAR_FORWARD ? TRACK_BASE_RPM : right_command,right_pwm,elapsed)) {
        Halt(CAR_ENCODER_FAULT); return;
    }
    if(state==CAR_FORWARD) {
        if((uint32_t)(now-phase_started)>=TRACK_FORWARD_TEST_MAX_MS) HaltReason(TRACK_STOP_FORWARD_TIME);
        else SetPWM(FORWARD_PWM,FORWARD_PWM);
        return;
    }
    dt=elapsed*0.001f; FilterSpeed(left_counts,right_counts,elapsed,dt);
    if(state==CAR_SPEED_TEST) {
        if((uint32_t)(now-phase_started)>=TRACK_BENCH_MAX_MS) HaltReason(TRACK_STOP_BENCH_TIME);
        else DriveTargets(elapsed,dt);
        return;
    }
    if(RecoveryStep(now,elapsed,count,sensor_raw_error,left_mm,right_mm,yaw)) return;
    if(state==CAR_CORNER_APPROACH) { Approach(now,elapsed,dt,forward_mm,yaw); return; }
    if(state==CAR_TURN_FAST || state==CAR_TURN_SLOW || state==CAR_SEARCH || state==CAR_ALIGN || state==CAR_EXIT)
        turn_angle+=yaw;
    switch(state) {
        case CAR_TURN_FAST: case CAR_TURN_SLOW: TurnCorner(now,elapsed,dt,count,sensor_raw_error); break;
        case CAR_SEARCH: Scan(now,elapsed,dt,count,sensor_raw_error); break;
        case CAR_ALIGN: Align(now,elapsed,dt,count,error,forward_mm); break;
        case CAR_EXIT: ExitLine(now,elapsed,dt,count,error,forward_mm); break;
        default: TrackLine(now,elapsed,dt,count,error,forward_mm); break;
    }
}
static int16_t Debug10(float value)
{
    return (int16_t)Limit(value*10.0f,-32767.0f,32767.0f);
}
void Track_GetDebug(TrackDebug *sample)
{
    if(!sample) return;
    sample->now_ms=Track_Now(); sample->state=(uint8_t)state; sample->mode=(uint8_t)motion_mode;
    sample->control_elapsed_ms=control_elapsed_ms; sample->max_control_elapsed_ms=max_control_elapsed_ms;
    sample->sensors=sensor_mask; sample->line_error10=(int8_t)Debug10(observed_error);
    sample->turn_hint=turn_hint; sample->corner_dir=corner_dir; sample->confidence=corner_confidence;
    sample->left_target10=Debug10(left_command); sample->right_target10=Debug10(right_command);
    sample->left_request10=Debug10(left_target); sample->right_request10=Debug10(right_target);
    sample->left_actual10=Debug10(left_filtered); sample->right_actual10=Debug10(right_filtered);
    sample->left_pwm=left_pwm; sample->right_pwm=right_pwm;
    sample->angle_deg10=Debug10(turn_angle*57.2957795f);
    sample->approach_mm=(uint16_t)Limit(approach_mm,0.0f,65535.0f);
    sample->approach_goal_mm=(uint16_t)Limit(approach_goal_mm,0.0f,65535.0f);
    sample->recovery_active=recovery_active; sample->recovery_attempts=recovery_attempts;
    sample->recovery_age_ms=recovery_active ? sample->now_ms-recovery_start_ms : 0;
    sample->recovery_angle_deg10=(uint16_t)Limit(recovery_total_angle*572.957795f,0.0f,65535.0f);
    sample->recovery_distance_mm=(uint16_t)Limit(recovery_distance,0.0f,65535.0f);
    sample->stop_reason=(uint8_t)stop_reason;
}
/* OLED_Show*只写显存，真正I2C传输在下面的空闲切片内执行。 */
static void Display(void)
{
#if TRACK_OLED_ENABLED
    uint8_t i,row=display_row,page=(uint8_t)((Track_Now()/TRACK_OLED_PAGE_MS)%3U);
    char *name;
    display_row=(uint8_t)((display_row+1U)%4U);
    OLED_ShowString((uint8_t)(row+1U),1,"                ");
    if(row==0) {
        switch(state) {
            case CAR_TRACK: name="TRACK"; break;
            case CAR_CORNER_CONFIRM: name="CONF "; break;
            case CAR_CORNER_APPROACH: name="APPR "; break;
            case CAR_TURN_FAST: name="FAST "; break;
            case CAR_TURN_SLOW: name="SLOW "; break;
            case CAR_ALIGN: name="ALIGN"; break;
            case CAR_EXIT: name="EXIT "; break;
            case CAR_SEARCH: name="SRCH "; break;
            case CAR_FORWARD: name="FWD  "; break;
            case CAR_SPEED_TEST: name="TEST "; break;
            case CAR_ENCODER_FAULT: name="ENC! "; break;
            case CAR_CONTROL_FAULT: name="TIME!"; break;
            case CAR_STOP: name="STOP "; break;
            case CAR_NO_LINE: name="NOIR "; break;
            default: name="READY"; break;
        }
        OLED_ShowString(1,1,name);
        if(!Track_IsRunning()) {
            OLED_ShowString(1,7,"R"); OLED_ShowSignedNum(1,8,stop_reason,2);
        } else {
            OLED_ShowString(1,7,"H"); OLED_ShowSignedNum(1,8,turn_hint,1);
            OLED_ShowString(1,12,"D"); OLED_ShowSignedNum(1,13,corner_dir,1);
        }
    } else if(row==1) {
        OLED_ShowString(2,1,"IR:");
        for(i=0;i<8;++i) OLED_ShowChar(2,(uint8_t)(4U+i),(sensor_mask&(1U<<i)) ? '1' : '0');
        OLED_ShowChar(2,12,'E'); OLED_ShowSignedNum(2,13,(int32_t)observed_error,1);
    } else if(row==2) {
        if(state==CAR_FORWARD) {
            OLED_ShowChar(3,1,'V'); OLED_ShowSignedNum(3,2,(int32_t)Limit(left_rpm,-999.0f,999.0f),3);
            OLED_ShowSignedNum(3,9,(int32_t)Limit(right_rpm,-999.0f,999.0f),3);
        } else if(page==0 || state==CAR_SPEED_TEST) {
            OLED_ShowChar(3,1,'T'); OLED_ShowSignedNum(3,2,(int32_t)left_command,3);
            OLED_ShowSignedNum(3,9,(int32_t)right_command,3);
        } else if(page==1) {
            OLED_ShowChar(3,1,'P'); OLED_ShowSignedNum(3,2,left_pwm,3);
            OLED_ShowSignedNum(3,9,right_pwm,3);
        } else {
            OLED_ShowChar(3,1,'A'); OLED_ShowSignedNum(3,2,(int32_t)(turn_angle*57.2957795f),3);
            OLED_ShowChar(3,8,'D'); OLED_ShowSignedNum(3,9,(int32_t)approach_mm,3);
            OLED_ShowChar(3,14,'N'); OLED_ShowChar(3,15,(char)('0'+recovery_attempts));
        }
    } else if(state==CAR_FORWARD || state==CAR_READY) {
        OLED_ShowChar(4,1,'C');
        OLED_ShowSignedNum(4,2,(int32_t)Limit((float)left_total_counts,-99999.0f,99999.0f),5);
        OLED_ShowSignedNum(4,9,(int32_t)Limit((float)right_total_counts,-99999.0f,99999.0f),5);
    } else {
        OLED_ShowChar(4,1,'V');
        OLED_ShowSignedNum(4,2,(int32_t)left_filtered,3);
        OLED_ShowSignedNum(4,9,(int32_t)right_filtered,3);
    }
#endif
}
void Track_Task(void)
{
    uint32_t now=Track_Now(),elapsed=now-last_control;
    if(elapsed>=TRACK_PERIOD_MS) {
        control_elapsed_ms=elapsed;
        if(elapsed>max_control_elapsed_ms) max_control_elapsed_ms=elapsed;
        last_control=now;
#if TRACK_TRACE_ENABLE
        trace_in_control=1;
#endif
        Control(now,elapsed);
#if TRACK_TRACE_ENABLE
        trace_in_control=0; TraceWrite(now);
#endif
    }
    now=Track_Now(); /* 控制计算可能跨毫秒，刷新前重新核对剩余时间。 */
#if TRACK_DEBUG
    if((uint32_t)(now-last_debug)>=TRACK_DEBUG_PERIOD_MS) {
        TrackDebug sample; last_debug=now; Track_GetDebug(&sample); track_debug=sample;
    }
#endif
#if TRACK_OLED_ENABLED
    if((uint32_t)(now-last_control)>=TRACK_PERIOD_MS-TRACK_OLED_IDLE_MARGIN_MS) return;
    if((uint32_t)(now-last_display)>=TRACK_OLED_ROW_MS) { last_display=now; Display(); }
    now=Track_Now();
    if((uint32_t)(now-last_control)<TRACK_PERIOD_MS-TRACK_OLED_IDLE_MARGIN_MS &&
       (uint32_t)(now-last_oled_service)>=TRACK_OLED_SERVICE_MS) {
        last_oled_service=now; OLED_Service(TRACK_OLED_SERVICE_BYTES);
    }
#endif
}
void Track_Init(void)
{
    TIM_TimeBaseInitTypeDef timer; NVIC_InitTypeDef irq; RCC_ClocksTypeDef clocks;
    uint32_t timer_clock; float error; uint8_t i;
    milliseconds=last_control=last_display=last_oled_service=0; display_row=0;
    control_elapsed_ms=max_control_elapsed_ms=0;
#if TRACK_DEBUG
    last_debug=0;
#endif
    left_rpm=right_rpm=0.0f; left_total_counts=right_total_counts=0;
    ResetRuntime(); Gray_Init(); ReadLine(&error);
    for(i=0;i<4;++i) Display();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); RCC_GetClocksFreq(&clocks);
    /* APB1 分频时，定时器时钟为总线时钟的两倍。 */
    timer_clock=clocks.PCLK1_Frequency;
    if(clocks.PCLK1_Frequency!=clocks.HCLK_Frequency) timer_clock*=2U;
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler=(uint16_t)(timer_clock/1000000U-1U); timer.TIM_Period=1000U-1U;
    TIM_TimeBaseInit(TIM3,&timer); TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
    irq.NVIC_IRQChannel=TIM3_IRQn; irq.NVIC_IRQChannelPreemptionPriority=2;
    irq.NVIC_IRQChannelSubPriority=0; irq.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&irq); TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); TIM_Cmd(TIM3,ENABLE);
}

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
typedef enum {
    CAR_READY=0, CAR_RUN=1, CAR_SEARCH=2, CAR_NO_LINE=4, CAR_LOST_STOP=8,
    CAR_ENCODER_FAULT=9, CAR_FORWARD=10, CAR_CORNER=11, CAR_WIDE=12,
    CAR_APPROACH=14, CAR_EXIT=15
} CarState;
static CarState state=CAR_READY;
static volatile uint32_t milliseconds;
static uint32_t last_control, last_display, phase_started;
static float last_error, error_filtered, left_rpm, right_rpm;
static float left_filtered, right_filtered, left_target, right_target;
static float left_command, right_command;
static int8_t left_pwm, right_pwm;
static uint8_t sensor_mask, normal_line_width=2, display_row;
static PID_TypeDef left_pid, right_pid, line_pid;
static float load_compensation[2];
/* 每个车轮独立记录故障和助推，避免左右状态互相干扰。 */
static uint32_t no_pulse_ms[2], reverse_ms[2], still_ms[2];
static float kick_pwm[2];
static uint8_t center_gap;
/* D: 1/2 左/右无脉冲，3/4 左/右反向，5～11 为各恢复阶段超时/超角度。 */
static uint8_t stop_detail;
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
static uint32_t recovery_started;
static uint8_t recovering;
static uint32_t width_learn_ms;
static uint8_t width_candidate;

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
static uint8_t ReadLine(float *error)
{
    uint8_t i=0,first,n,has_center=0,has_left=0,has_right=0;
    float left_position=0.0f,right_position=0.0f;
    float position,distance,best=100.0f;
    sensor_mask=Gray_Read(); *error=0.0f; side_observed=0;
    while(i<8) {
        if(!(sensor_mask&(1U<<i))) { ++i; continue; }
        first=i; n=0;
        while(i<8 && (sensor_mask&(1U<<i))) { ++n; ++i; }
        position=(float)(2*first+n-1)-7.0f;
        if(first<=4 && i>3) has_center=1;
        if(i<=3) { has_left=1; left_position=position; }
        if(first>=5) { has_right=1; right_position=position; }
        distance=Abs(position-last_error);
        if(distance<best) { best=distance; *error=position; }
    }
    if(has_center && has_left!=has_right) {
        side_observed=has_left ? -1 : 1;
        side_error=has_left ? left_position : right_position;
    }
    return Bits(sensor_mask);
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
           Abs(error)<=1.5f && Contiguous(sensor_mask);
}
/* 首帧严格居中后，邻帧允许窄线稍偏；边缘及分离黑区不能补位。 */
static uint8_t NearCenter(uint8_t count,float error)
{
    return count && count<=normal_line_width+1U && !(sensor_mask&0x81U) &&
           Contiguous(sensor_mask) && Abs(error)<=3.0f;
}
static void ClearWide(void)
{
    wide_active=wide_strong=wide_peak_count=0; wide_ms=0; wide_dir=0; wide_travel_mm=wide_last_mm=0.0f;
    wide_center_ms=wide_tail_ms=wide_gap_ms=0; wide_center_mm=0.0f;
}
/* 只保存已运动且未明显超速时的正向负载补偿，不保存误差和微分。
 * 正反转均按速度大小控制，补偿也按大小使用；手动停车会全部清空。
 */
static void SaveLoad(uint8_t wheel,PID_TypeDef *pid)
{
    if(drive_sign[wheel] && pid->integral>0.0f && Abs(pid->actual)>=8.0f &&
       Abs(pid->actual)<=Abs(pid->target)+TRACK_SPEED_COAST_MARGIN)
        load_compensation[wheel]=Limit(load_compensation[wheel]+pid->integral,0.0f,TRACK_LOAD_COMP_MAX_PWM);
}
/* 步骤 2：清空阶段误差和命令；运动阶段保留有界负载补偿，减少出弯重起冲击。 */
static void ResetPID(void)
{
    if(Track_IsRunning()) { SaveLoad(0,&left_pid); SaveLoad(1,&right_pid); }
    PID_Init(&left_pid,TRACK_SPEED_KP,TRACK_SPEED_KI,TRACK_SPEED_KD,TRACK_SPEED_I_LIMIT,TRACK_SPEED_D_TAU);
    PID_Init(&right_pid,TRACK_SPEED_KP,TRACK_SPEED_KI,TRACK_SPEED_KD,TRACK_SPEED_I_LIMIT,TRACK_SPEED_D_TAU);
    PID_Init(&line_pid,TRACK_LINE_KP,TRACK_LINE_KI,TRACK_LINE_KD,TRACK_LINE_I_LIMIT,TRACK_LINE_D_TAU);
    left_filtered=left_rpm; right_filtered=right_rpm;
    left_target=right_target=left_command=right_command=0.0f;
    no_pulse_ms[0]=no_pulse_ms[1]=reverse_ms[0]=reverse_ms[1]=0;
    still_ms[0]=still_ms[1]=0; kick_pwm[0]=kick_pwm[1]=0.0f;
    drive_sign[0]=drive_sign[1]=0;
    steering_urgent=0;
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
    return state==CAR_RUN || state==CAR_WIDE || state==CAR_SEARCH || state==CAR_FORWARD ||
           state==CAR_APPROACH || state==CAR_CORNER || state==CAR_EXIT;
}
void Track_Stop(void)
{
    SetPWM(0,0); state=CAR_READY; last_error=error_filtered=0.0f;
    load_compensation[0]=load_compensation[1]=0.0f;
    ClearWide(); lost_ms=centered_ms=0; approach_mm=approach_goal_mm=turn_angle=rearm_mm=0.0f;
    corner_dir=last_turn_dir=0; sweep_number=center_samples=center_gap=0;
    side_candidate=side_observed=turn_hint=0; side_ms=hint_seen=0;
    edge_ms=0; edge_dir=0;
    edge_align_active=0;
    edge_best_error=0.0f; recovery_started=0; recovering=0;
    width_learn_ms=0; width_candidate=0;
    corner_armed=1; normal_line_width=2; ResetPID();
}
static void Halt(CarState reason,uint8_t detail) { Track_Stop(); state=reason; stop_detail=detail; }
/* KEY5 人工直行检查不依赖黑线和速度闭环，保留固定 30% PWM。
 * 计数在开始时清零，方便核对两轮实际向前时的编码器正负号。
 */
void Track_StartForward(void)
{
    Track_Stop(); Encoder_Clear(1); Encoder_Clear(2);
    left_rpm=right_rpm=0.0f; ResetPID(); left_total_counts=right_total_counts=0;
    last_control=Track_Now(); state=CAR_FORWARD; SetPWM(FORWARD_PWM,FORWARD_PWM);
}
void Track_Start(void)
{
    float error; uint8_t count;
    Track_Stop(); stop_detail=0; count=ReadLine(&error);
    if(!count) { state=CAR_NO_LINE; return; }
    if(Centered(count,error) && count>normal_line_width) normal_line_width=count;
    Encoder_Clear(1); Encoder_Clear(2); left_rpm=right_rpm=0.0f; ResetPID();
    left_total_counts=right_total_counts=0; last_error=error_filtered=error;
    last_control=Track_Now(); state=CAR_RUN;
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
static int8_t SpeedOutput(uint8_t wheel,PID_TypeDef *pid,float target,float rpm,
                          float raw_rpm,uint32_t elapsed,float dt,float feedforward,float offset)
{
    float out,minimum=0.0f;
    int8_t sign=target<0.0f ? -1 : 1;
    if(target<0.0f) target=-target;
    if(target<TRACK_MIN_DRIVE_RPM) {
        SaveLoad(wheel,pid);
        PID_Reset(pid); still_ms[wheel]=0; kick_pwm[wheel]=0.0f;
        drive_sign[wheel]=0; return 0;
    }
    if(drive_sign[wheel]!=sign) {
        SaveLoad(wheel,pid);
        PID_Reset(pid); still_ms[wheel]=0; kick_pwm[wheel]=0.0f;
        drive_sign[wheel]=sign;
    }
    pid->target=sign*target; pid->actual=rpm; rpm*=sign; raw_rpm*=sign;
    /* 无计数时逐帧提高 PWM 下限；读到正向转动立即撤回，可反复重新助推。
     * 不把助推 PWM 灌入积分，恢复后仍由原速度环接管。
     */
    if(raw_rpm>=8.0f) {
        still_ms[wheel]=0; kick_pwm[wheel]=0.0f;
    } else if(raw_rpm>-6.0f && raw_rpm<6.0f) {
        still_ms[wheel]+=elapsed;
        if(still_ms[wheel]>=TRACK_STALL_KICK_MS) {
            kick_pwm[wheel]=kick_pwm[wheel]<TRACK_BOOST_PWM ? TRACK_BOOST_PWM :
                            kick_pwm[wheel]+TRACK_KICK_STEP_PWM;
            if(kick_pwm[wheel]>TRACK_KICK_MAX_PWM) kick_pwm[wheel]=TRACK_KICK_MAX_PWM;
        }
    } else { still_ms[wheel]=0; kick_pwm[wheel]=0.0f; }
    minimum=kick_pwm[wheel];
    out=PID_Step(pid,target-rpm,dt,offset+load_compensation[wheel]+feedforward*target,minimum,
                 minimum>(float)TRACK_MAX_PWM ? minimum : (float)TRACK_MAX_PWM);
    /* 上一段的正积分不能在目标已降低时继续推快轮；滑行降速后再接回闭环。 */
    if(rpm>target+TRACK_SPEED_COAST_MARGIN) { pid->integral*=0.5f; out=0.0f; }
    return sign*(int8_t)(out+0.5f);
}
static uint8_t EncoderFault(uint8_t wheel,int16_t pulses,float rpm,float command,int8_t previous_pwm,uint32_t elapsed)
{
    /* 只监测被命令运动的轮子；内轮停止和偶发调度空档不算故障。 */
    if(command<0.0f) { command=-command; rpm=-rpm; }
    if(previous_pwm<0) previous_pwm=-previous_pwm;
    if(command<TRACK_MIN_DRIVE_RPM || previous_pwm<ENCODER_FAULT_MIN_PWM || elapsed>2U*TRACK_PERIOD_MS) {
        no_pulse_ms[wheel]=reverse_ms[wheel]=0; return 0;
    }
    if(!pulses) no_pulse_ms[wheel]+=elapsed; else no_pulse_ms[wheel]=0;
    if(rpm<-6.0f) reverse_ms[wheel]+=elapsed; else reverse_ms[wheel]=0;
    /* 无脉冲仍累计 700 ms，但必须已经助推封顶才停车；反向检测独立保留。 */
    if(no_pulse_ms[wheel]>=ENCODER_NO_PULSE_MS && kick_pwm[wheel]>=TRACK_KICK_MAX_PWM)
        return (uint8_t)(1U+wheel);
    if(reverse_ms[wheel]>=ENCODER_REVERSE_MS) return (uint8_t)(3U+wheel);
    return 0;
}
static void DriveTargets(uint32_t elapsed,float dt)
{
    float step=(steering_urgent ? TRACK_EDGE_SLEW_RPM_PER_S : TRACK_TARGET_SLEW_RPM_PER_S)*dt;
    if(elapsed>2U*TRACK_PERIOD_MS) elapsed=TRACK_PERIOD_MS;
    /* 每周期限制轮速变化，防止数字探头跳一格时车头猛甩。 */
    left_command+=Limit(left_target-left_command,-step,step);
    right_command+=Limit(right_target-right_command,-step,step);
    SetPWM(SpeedOutput(0,&left_pid,left_command,left_filtered,left_rpm,elapsed,dt,LEFT_SPEED_FEEDFORWARD,LEFT_DRIVE_OFFSET_PWM),
           SpeedOutput(1,&right_pid,right_command,right_filtered,right_rpm,elapsed,dt,RIGHT_SPEED_FEEDFORWARD,RIGHT_DRIVE_OFFSET_PWM));
}
/* 步骤 4：横带与偏侧弯线分开处理。全黑/六路以上强横带才冻结位置纠偏；
 * 四五路弱线索只记忆并减速，仍允许跟随侧边。不能一见宽线就盲目前进。
 */
static uint8_t ObserveWide(uint32_t now,uint32_t elapsed,uint8_t count,float forward_mm)
{
#if TRACK_ENABLE_CORNERS
    uint8_t broad;
    if(!corner_armed || elapsed>2U*TRACK_PERIOD_MS) { ClearWide(); return 0; }
    broad=count>=4 && count>=normal_line_width+2U && Contiguous(sensor_mask) &&
          (sensor_mask&0x81U) && (sensor_mask&0x18U);
    if(sensor_mask==0xFFU) broad=1;
    if(wide_active && forward_mm>0.0f) wide_travel_mm+=forward_mm;
    if(broad) {
        edge_ms=0; edge_dir=0; /* 宽带打断边缘连续计时，不能跨宽带凑满600ms。 */
        if(!wide_active) { ClearWide(); wide_active=1; }
        wide_ms+=elapsed; wide_gap_ms=0; wide_last_seen=now; wide_last_mm=wide_travel_mm;
        if(count>wide_peak_count) wide_peak_count=count;
        wide_center_ms=wide_tail_ms=0; wide_center_mm=0.0f;
        if(count>=6) wide_strong=1;
        /* 单侧扩展才有左右线索；八路全黑不能强行推断方向。 */
        if((sensor_mask&0x01U) && !(sensor_mask&0x80U)) wide_dir=-1;
        else if((sensor_mask&0x80U) && !(sensor_mask&0x01U)) wide_dir=1;
        lost_ms=0;
        if(wide_strong) {
            state=CAR_WIDE; PID_Reset(&line_pid);
            left_target=right_target=CORNER_APPROACH_RPM; return 1;
        }
        return 0;
    }
    /* 反光/接缝可以在横带中漏一两帧，但单帧宽线仍不能触发长推进。
     * 只在40ms窗口内保留候选；更长间隔必须重新累计宽帧。
     */
    if(wide_active && wide_ms<CORNER_CONFIRM_MS) {
        wide_gap_ms+=elapsed;
        if(wide_gap_ms>CORNER_CONFIRM_GAP_MS) ClearWide();
        return 0;
    }
    /* 斜着穿过较宽拐角时，末尾可能由六路渐减到五路、四路甚至一路。
     * 这些与已知方向一致的边缘黑区仍属于横线尾部，要延长记忆并补足宽度。
     * 否则宽线尾部耗掉 200 ms 记忆，最后全白时又会错误地提前旋转。
     */
    if(wide_active && count && Contiguous(sensor_mask)) {
        int8_t tail_dir=0;
        if((sensor_mask&0x01U) && !(sensor_mask&0x80U)) tail_dir=-1;
        else if((sensor_mask&0x80U) && !(sensor_mask&0x01U)) tail_dir=1;
        if(tail_dir && (!wide_dir || tail_dir==wide_dir)) {
            wide_dir=tail_dir; wide_last_seen=now; wide_last_mm=wide_travel_mm;
        }
    }
    if(wide_active && (uint32_t)(now-wide_last_seen)>CORNER_MEMORY_MS) ClearWide();
#else
    (void)now; (void)elapsed; (void)count; (void)forward_mm;
#endif
    return 0;
}
/* 普通弯道：边缘探头已看到线时立即减速并朝该侧纠偏。
 * 中间位置保留原滤波；边缘采用较快滤波与轮速变化，避免等全白才转。
 */
static void FollowLine(uint32_t now,float error,float dt,float speed_cap)
{
    float magnitude,base,correction,tau=TRACK_LINE_FILTER_TAU;
    float ratio=TRACK_MAX_CORRECTION_RATIO;
    float control_error=error;
    last_error=error;
    steering_urgent=Abs(error)>=TRACK_EDGE_ERROR;
    if(steering_urgent) { tau=TRACK_EDGE_FILTER_TAU; ratio=TRACK_EDGE_CORRECTION_RATIO; }
    if(Abs(error)>=3.0f) { turn_hint=error<0.0f ? -1 : 1; hint_seen=now; }
    if(Abs(error)<=1.5f) {
        control_error=Abs(error)>TRACK_CENTER_DEADBAND ?
                      (error<0.0f ? -1.0f : 1.0f)*(Abs(error)-TRACK_CENTER_DEADBAND) : 0.0f;
    }
    error_filtered+=dt/(tau+dt)*(control_error-error_filtered);
    magnitude=Abs(error_filtered);
    if(steering_urgent && Abs(error)>magnitude) magnitude=Abs(error);
    base=TRACK_BASE_RPM-TRACK_BEND_SLOWDOWN*magnitude;
    base=Limit(base,TRACK_BEND_MIN_RPM,speed_cap);
    correction=PID_Step(&line_pid,error_filtered,dt,0.0f,-base*ratio,base*ratio);
    if(steering_urgent) {
        /* 滤波尚未追上新的边缘位置时，也不能给出反方向或近似直行命令。 */
        if(error<0.0f && correction>-base*TRACK_EDGE_MIN_CORRECTION) correction=-base*TRACK_EDGE_MIN_CORRECTION;
        if(error>0.0f && correction< base*TRACK_EDGE_MIN_CORRECTION) correction= base*TRACK_EDGE_MIN_CORRECTION;
    }
    left_target=base+correction; right_target=base-correction;
}
static void BeginApproach(uint32_t now)
{
    float half_width=wide_last_mm*0.5f,after_wide=wide_travel_mm-wide_last_mm;
    /* 宽线短于一个采样周期时长度难测，使用半线宽初值补偿。 */
    if(half_width<2.0f) half_width=LINE_HALF_WIDTH_MM;
    half_width=Limit(half_width,2.0f,45.0f);
    approach_goal_mm=SENSOR_FRONT_OFFSET_MM-half_width-after_wide+CORNER_ADVANCE_TRIM_MM;
    approach_goal_mm=Limit(approach_goal_mm,0.0f,SENSOR_FRONT_OFFSET_MM+50.0f);
    corner_dir=wide_dir ? wide_dir : (last_turn_dir ? last_turn_dir : CORNER_DEFAULT_DIR);
    ClearWide(); corner_armed=0; centered_ms=0; rearm_mm=0.0f;
    side_candidate=0; side_ms=0;
    edge_ms=0; edge_dir=0;
    approach_mm=0.0f; turn_angle=0.0f; phase_started=now; state=CAR_APPROACH;
    PID_Reset(&line_pid); left_target=right_target=CORNER_APPROACH_RPM;
}
static void BeginScan(uint32_t now,int8_t direction,uint8_t is_corner)
{
    ResetPID(); SetPWM(0,0); ClearWide(); state=is_corner==1 ? CAR_CORNER : CAR_SEARCH;
    edge_align_active=is_corner==2;
    corner_dir=direction ? direction : CORNER_DEFAULT_DIR;
    phase_started=now; if(is_corner!=1) turn_angle=0.0f; /* 直角保留推进偏航，以原直线方向为基准 */
    sweep_number=1; center_samples=center_gap=0; lost_ms=0;
    side_candidate=0; side_ms=0;
    edge_ms=0; edge_dir=0;
}
/* 有线贴边接管：保留两轮速度环，内轮停车、外轮缓慢推进，避免停车后反扫。 */
static void BeginEdgeAlign(uint32_t now,int8_t direction)
{
    ClearWide(); PID_Reset(&line_pid); state=CAR_SEARCH; edge_align_active=1;
    corner_dir=direction; phase_started=now; turn_angle=0.0f;
    sweep_number=1; center_samples=center_gap=0; lost_ms=0; side_candidate=0; side_ms=0;
    edge_ms=0; edge_dir=0;
    left_target=direction>0 ? TRACK_EDGE_PIVOT_RPM : 0.0f;
    right_target=direction<0 ? TRACK_EDGE_PIVOT_RPM : 0.0f;
}
/* 普通救线接回低速循迹时不清空速度积分、不重复刹停和助推。 */
static void ResumeLine(uint32_t now,uint32_t elapsed,float dt,float error)
{
    state=CAR_RUN; edge_align_active=0; recovering=1; recovery_started=now;
    PID_Reset(&line_pid); last_error=error; error_filtered=0.0f;
    lost_ms=0; edge_ms=0; edge_dir=0;
    FollowLine(now,error,dt,TRACK_RECOVERY_RPM); DriveTargets(elapsed,dt);
}
/* 步骤 5：有限角度扫描。第一侧找不到就扫另一侧，仍找不到则停车。
 * 反向时不清零转角，否则无法判断是否扫到原朝向的另一侧。
 */
static void Scan(uint32_t now,uint32_t elapsed,float dt,uint8_t count,float error)
{
    uint8_t is_corner=state==CAR_CORNER;
    float min_angle=is_corner ? CORNER_MIN_ANGLE_RAD : 0.0f;
    float max_angle=is_corner ? CORNER_SCAN_LIMIT_RAD : TRACK_SEARCH_SCAN_RAD;
    float progress=-corner_dir*turn_angle,rpm=is_corner ? CORNER_RPM : TRACK_SEARCH_RPM;
    uint32_t timeout=is_corner ? CORNER_TURN_MAX_MS : TRACK_SEARCH_MAX_MS;
    if(edge_align_active) {
        /* 没有读到正中但已经跨到另一侧时，也交回连续PD，不继续转过头。 */
        uint8_t crossed=count && Contiguous(sensor_mask) && count<=normal_line_width+1U &&
                        corner_dir*error<=-3.0f;
        if(Centered(count,error) || crossed) ++center_samples; else center_samples=0;
        if(center_samples>=CORNER_CENTER_SAMPLES) { ResumeLine(now,elapsed,dt,error); return; }
        if(progress>=TRACK_EDGE_ALIGN_LIMIT_RAD ||
           (uint32_t)(now-phase_started)>=TRACK_EDGE_ALIGN_MAX_MS) { Halt(CAR_LOST_STOP,progress>=TRACK_EDGE_ALIGN_LIMIT_RAD ? 10 : 11); return; }
        rpm=count && Abs(error)<=3.0f ? CORNER_ALIGN_RPM : TRACK_EDGE_PIVOT_RPM;
        left_target=corner_dir>0 ? rpm : 0.0f;
        right_target=corner_dir<0 ? rpm : 0.0f;
        DriveTargets(elapsed,dt); return;
    }
    if((uint32_t)(now-phase_started)<CORNER_SETTLE_MS) { SetPWM(0,0); return; }
    if((uint32_t)(now-phase_started)>=timeout) { Halt(CAR_LOST_STOP,is_corner ? 6 : 8); return; }
    /* 首帧仍须严格居中；之后可容一帧漏读，后续近中心帧才增加确认数。 */
    if(progress>=min_angle && (Centered(count,error) || (center_samples && NearCenter(count,error)))) {
        ++center_samples; center_gap=0;
    } else if(center_samples && center_gap<1U) ++center_gap;
    else { center_samples=0; center_gap=0; }
    if(center_samples>=CORNER_CENTER_SAMPLES) {
        if(!is_corner) { ResumeLine(now,elapsed,dt,error); return; }
        if(is_corner) last_turn_dir=corner_dir;
        SetPWM(0,0); ResetPID(); state=CAR_EXIT; phase_started=now;
        last_error=error_filtered=error; centered_ms=0; rearm_mm=0.0f; return;
    }
    if(progress>=max_angle) {
        if(sweep_number>=2) { Halt(CAR_LOST_STOP,is_corner ? 7 : 9); return; }
        ++sweep_number; corner_dir=-corner_dir; center_samples=center_gap=0;
        ResetPID(); SetPWM(0,0); return;
    }
    /* 中间探头开始看到线时放慢旋转，给中心连续确认留时间。 */
    if(count && count<=5 && !(sensor_mask&0x81U) && progress>=min_angle) rpm=CORNER_ALIGN_RPM;
    left_target=corner_dir*rpm; right_target=-corner_dir*rpm; DriveTargets(elapsed,dt);
}
/* 步骤 6：一轮控制先测速度，再决定阶段，最后生成 PWM。 */
static void Control(uint32_t now,uint32_t elapsed)
{
    float error,dt,alpha,left_mm,right_mm,forward_mm,scale,largest;
    int16_t raw_left,raw_right,left_counts,right_counts;
    uint8_t count,fault;
    Encoder_GetPair(&raw_left,&raw_right);
#if DRIVE_PAIRS_SWAPPED
    left_counts=RIGHT_ENCODER_SIGN*raw_right; right_counts=LEFT_ENCODER_SIGN*raw_left;
    left_mm=(float)left_counts*3.14159265f*WHEEL_DIAMETER_MM/RIGHT_COUNTS_PER_REV;
    right_mm=(float)right_counts*3.14159265f*WHEEL_DIAMETER_MM/LEFT_COUNTS_PER_REV;
    left_rpm=(float)left_counts*60000.0f/(RIGHT_COUNTS_PER_REV*elapsed);
    right_rpm=(float)right_counts*60000.0f/(LEFT_COUNTS_PER_REV*elapsed);
#else
    left_counts=LEFT_ENCODER_SIGN*raw_left; right_counts=RIGHT_ENCODER_SIGN*raw_right;
    left_mm=(float)left_counts*3.14159265f*WHEEL_DIAMETER_MM/LEFT_COUNTS_PER_REV;
    right_mm=(float)right_counts*3.14159265f*WHEEL_DIAMETER_MM/RIGHT_COUNTS_PER_REV;
    left_rpm=(float)left_counts*60000.0f/(LEFT_COUNTS_PER_REV*elapsed);
    right_rpm=(float)right_counts*60000.0f/(RIGHT_COUNTS_PER_REV*elapsed);
#endif
    left_total_counts+=left_counts; right_total_counts+=right_counts;
    forward_mm=(left_mm+right_mm)*0.5f;
    if(state==CAR_FORWARD) { SetPWM(FORWARD_PWM,FORWARD_PWM); return; }
    count=ReadLine(&error);
    steering_urgent=0;
    if(!Track_IsRunning()) { SetPWM(0,0); return; }
    if(elapsed>2U*TRACK_PERIOD_MS) { ResetPID(); ClearWide(); lost_ms=0; center_samples=center_gap=0; }
    fault=EncoderFault(0,left_counts,left_rpm,left_command,left_pwm,elapsed);
    if(!fault) fault=EncoderFault(1,right_counts,right_rpm,right_command,right_pwm,elapsed);
    if(fault) { Halt(CAR_ENCODER_FAULT,fault); return; }
    dt=(elapsed>2U*TRACK_PERIOD_MS ? TRACK_PERIOD_MS : elapsed)*0.001f;
    alpha=dt/(TRACK_RPM_FILTER_TAU+dt);
    left_filtered+=alpha*(left_rpm-left_filtered); right_filtered+=alpha*(right_rpm-right_filtered);
    /* 推进时探头允许全白。只按轮子距离推进，轮轴未到位绝不开始旋转。 */
    if(state==CAR_APPROACH) {
        approach_mm+=forward_mm;
        turn_angle+=(right_mm-left_mm)/AXLE_TRACK_MM; /* 从原直线方向累计推进偏航 */
        if(approach_mm>=approach_goal_mm) { BeginScan(now,corner_dir,1); return; }
        if((uint32_t)(now-phase_started)>=CORNER_APPROACH_MAX_MS) { Halt(CAR_LOST_STOP,5); return; }
        {
            /* 正偏航表示左偏：左轮加速、右轮减速以保持直线。 */
            float hold=Limit(TRACK_APPROACH_YAW_KP*turn_angle,-TRACK_APPROACH_YAW_MAX_RPM,TRACK_APPROACH_YAW_MAX_RPM);
            left_target=CORNER_APPROACH_RPM+hold; right_target=CORNER_APPROACH_RPM-hold;
        }
        DriveTargets(elapsed,dt); return;
    }
    if(state==CAR_CORNER || state==CAR_SEARCH) {
        turn_angle+=(right_mm-left_mm)/AXLE_TRACK_MM; Scan(now,elapsed,dt,count,error); return;
    }
    if(state==CAR_EXIT) {
        SetPWM(0,0);
        if((uint32_t)(now-phase_started)<CORNER_SETTLE_MS) return;
        if(!count) { BeginScan(now,-corner_dir,0); return; }
        ResetPID(); state=CAR_RUN; last_error=error_filtered=error;
    }
    ResolveSide(elapsed,&error);
    if(recovering && (uint32_t)(now-recovery_started)>=TRACK_RECOVERY_SOFT_MS) recovering=0;
    /* 离开上一角点 50 mm 且窄线稳定可见后重新识别；弯道不必强求正中两路。 */
    if(!corner_armed) {
        if(forward_mm>0.0f) rearm_mm+=forward_mm;
        if(count && count<=normal_line_width+1U && Contiguous(sensor_mask) &&
           Abs(error)<=5.0f && elapsed<=2U*TRACK_PERIOD_MS) centered_ms+=elapsed; else centered_ms=0;
        if(centered_ms>=CORNER_REARM_CENTER_MS && rearm_mm>=CORNER_REARM_TRAVEL_MM) corner_armed=1;
    }
    /* 角点会短暂出现中间四五路黑，不能立即把它当作正常线宽。
     * 起步按实际中心线确定宽度；运行中只允许稳定、逐级扩一格。
     */
    if(!wide_active && state==CAR_RUN && Centered(count,error) &&
       count>normal_line_width && count<=normal_line_width+1U && elapsed<=2U*TRACK_PERIOD_MS) {
        if(width_candidate!=count) { width_candidate=count; width_learn_ms=0; }
        width_learn_ms+=elapsed;
        if(width_learn_ms>=TRACK_WIDTH_LEARN_MS) { normal_line_width=count; width_learn_ms=0; }
    } else { width_candidate=0; width_learn_ms=0; }
    if(ObserveWide(now,elapsed,count,forward_mm)) { DriveTargets(elapsed,dt); return; }
    if(count) {
        lost_ms=0;
        /* 一帧中线不能抹去侧向角点。恢复正常线需连续时间和实际前行距离确认。 */
        if(wide_active) {
            if(Centered(count,error) && count<=normal_line_width) {
                wide_center_ms+=elapsed;
                if(forward_mm>0.0f) wide_center_mm+=forward_mm;
                if(wide_center_ms>=CORNER_CLEAR_CENTER_MS && wide_center_mm>=CORNER_CLEAR_TRAVEL_MM) ClearWide();
            } else { wide_center_ms=0; wide_center_mm=0.0f; }
            /* 弱宽区逐渐缩到侧边窄线，属于连续弯道；不再套用前置距离的直角推进。 */
            if(wide_active && !wide_strong && count<=normal_line_width+1U && Abs(error)>=4.0f) {
                wide_tail_ms+=elapsed;
                if(wide_tail_ms>=TRACK_SIDE_CONFIRM_MS) ClearWide();
            } else wide_tail_ms=0;
        }
        if(wide_active && wide_strong && !wide_center_ms && wide_ms>=CORNER_CONFIRM_MS) {
            edge_ms=0; edge_dir=0;
            state=CAR_WIDE; left_target=right_target=CORNER_APPROACH_RPM; DriveTargets(elapsed,dt); return;
        }
    } else {
        lost_ms+=elapsed;
        if(wide_active && wide_peak_count>=CORNER_MIN_ADVANCE_PROBES &&
           wide_ms>=CORNER_CONFIRM_MS && lost_ms>=CORNER_LOST_CONFIRM_MS) {
            BeginApproach(now); DriveTargets(elapsed,dt); return;
        }
        if(lost_ms<TRACK_LOST_GRACE_MS) {
            /* 漏读一两帧时先低速缓冲，不立刻原地转向。 */
            if(wide_active && wide_peak_count>=CORNER_MIN_ADVANCE_PROBES && wide_ms>=CORNER_CONFIRM_MS)
                left_target=right_target=CORNER_APPROACH_RPM;
            else {
                /* 共同缩放保留差速比例，不能把弯道两轮分别截成同一个速度。 */
                largest=Abs(left_target)>Abs(right_target) ? Abs(left_target) : Abs(right_target);
                scale=largest>TRACK_BEND_MIN_RPM ? TRACK_BEND_MIN_RPM/largest : 1.0f;
                left_target*=scale; right_target*=scale; steering_urgent=1;
            }
            DriveTargets(elapsed,dt); return;
        }
        BeginScan(now,last_error>=3.0f ? 1 : (last_error<=-3.0f ? -1 :
                  (turn_hint && (uint32_t)(now-hint_seen)<=TRACK_TURN_HINT_MS ? turn_hint :
                  (last_turn_dir ? last_turn_dir : CORNER_DEFAULT_DIR))),0); return;
    }
    state=CAR_RUN;
    /* 视频中最外一路连续有线数秒、内轮几乎停住，继续小轮速只会绕偏。
     * 持续最外侧无改善时内轮停、外轮有限推进对线，不刹停后反复反扫。
     * 强横带仍走原轴线推进流程，不在此提前转直角。
     */
    if(!wide_active && !recovering && count && count<=normal_line_width+1U &&
       Contiguous(sensor_mask) && Abs(error)>=TRACK_EDGE_ALIGN_ERROR && elapsed<=2U*TRACK_PERIOD_MS) {
        int8_t direction=error<0.0f ? -1 : 1;
        if(edge_dir!=direction) { edge_dir=direction; edge_ms=0; edge_best_error=Abs(error); }
        if(Abs(error)<edge_best_error-0.5f) { edge_ms=0; edge_best_error=Abs(error); }
        edge_ms+=elapsed;
        if(edge_ms>=TRACK_EDGE_ALIGN_MS) { BeginEdgeAlign(now,direction); DriveTargets(elapsed,dt); return; }
    } else { edge_ms=0; edge_dir=0; }
    if(sensor_mask==0xFFU) {
        /* 关闭直角识别时，全黑只作横条低速直行；不据此认定终点。 */
        PID_Reset(&line_pid); left_target=right_target=TRACK_BEND_MIN_RPM;
    } else {
        FollowLine(now,error,dt,recovering ? TRACK_RECOVERY_RPM :
                   (wide_active || side_candidate ? CORNER_APPROACH_RPM : TRACK_BASE_RPM));
    }
    DriveTargets(elapsed,dt);
}
/* 步骤 7：每次只刷新一行，分散软 I2C 工作量，减少耽误 20 ms 控制周期。
 * OLED 字库是 ASCII，状态中文含义在说明书里，代码注释使用中文。
 */
static void Display(void)
{
#if TRACK_OLED_ENABLED
    uint8_t i,row=display_row;
    int32_t lc,rc;
    display_row=(uint8_t)((display_row+1U)%4U);
    if(row==0) {
        switch(state) {
            case CAR_RUN: OLED_ShowString(1,1,"RUN  K1:STOP    "); break;
            case CAR_WIDE: OLED_ShowString(1,1,"WIDE:WAIT EXIT  "); break;
            case CAR_SEARCH: OLED_ShowString(1,1,edge_align_active ? "EDGE CRAWL      " : "SEARCH LIMITED  "); break;
            case CAR_NO_LINE: OLED_ShowString(1,1,"NO LINE! RETRY  "); break;
            case CAR_ENCODER_FAULT:
                OLED_ShowString(1,1,"ENC STOP D:     ");
                OLED_ShowChar(1,12,(char)('0'+stop_detail/10U));
                OLED_ShowChar(1,13,(char)('0'+stop_detail%10U)); break;
            case CAR_LOST_STOP:
                OLED_ShowString(1,1,"LOST STOP D:    ");
                OLED_ShowChar(1,13,(char)('0'+stop_detail/10U));
                OLED_ShowChar(1,14,(char)('0'+stop_detail%10U)); break;
            case CAR_APPROACH: OLED_ShowString(1,1,"AXLE ADVANCE    "); break;
            case CAR_CORNER: OLED_ShowString(1,1,corner_dir<0 ? "TURN LEFT       " : "TURN RIGHT      "); break;
            case CAR_EXIT: OLED_ShowString(1,1,"LINE REACQUIRED "); break;
            case CAR_FORWARD: OLED_ShowString(1,1,"FWD  K5:STOP    "); break;
            default: OLED_ShowString(1,1,"FIX4 K1:TRACK   "); break;
        }
    } else if(row==1) {
        OLED_ShowString(2,1,state==CAR_FORWARD ? "IR:NOT USED     " : "IR:             ");
        if(state!=CAR_FORWARD) for(i=0;i<8;++i) OLED_ShowChar(2,4+i,(sensor_mask&(1U<<i)) ? '1' : '0');
    } else if(row==2) {
        /* 约251计数/圈下，20ms单计数相当于约12RPM；显示控制用滤波轮速。
         * 人工直行与准备状态未运行速度滤波，仍显示当前采样值。
         */
        uint8_t direct=state==CAR_FORWARD || !Track_IsRunning();
        OLED_ShowString(3,1,"L:"); OLED_ShowSignedNum(3,3,(int32_t)(direct ? left_rpm : left_filtered),4);
        OLED_ShowString(3,9,"R:"); OLED_ShowSignedNum(3,11,(int32_t)(direct ? right_rpm : right_filtered),4);
    } else if(state==CAR_FORWARD || state==CAR_READY) {
        lc=left_total_counts; rc=right_total_counts;
        if(lc>99999) lc=99999;
        if(lc<-99999) lc=-99999;
        if(rc>99999) rc=99999;
        if(rc<-99999) rc=-99999;
        OLED_ShowString(4,1,"C:              ");
        OLED_ShowSignedNum(4,3,lc,5); OLED_ShowSignedNum(4,10,rc,5);
    } else {
        OLED_ShowString(4,1,"PWM:            "); OLED_ShowSignedNum(4,5,left_pwm,3); OLED_ShowSignedNum(4,10,right_pwm,3);
    }
#endif
}
void Track_Task(void)
{
    uint32_t now=Track_Now(),elapsed=now-last_control;
    if(elapsed>=TRACK_PERIOD_MS) { last_control=now; Control(now,elapsed); }
    /* 刚执行控制之后才刷一行，四行约 240 ms 刷新完一遍。 */
    if(elapsed>=TRACK_PERIOD_MS && (uint32_t)(now-last_display)>=60U) { last_display=now; Display(); }
}
void Track_Init(void)
{
    TIM_TimeBaseInitTypeDef timer; NVIC_InitTypeDef irq; RCC_ClocksTypeDef clocks;
    uint32_t timer_clock; float error; uint8_t i;
    milliseconds=last_control=last_display=0; display_row=0;
    left_rpm=right_rpm=0.0f; left_total_counts=right_total_counts=0;
    Track_Stop(); Gray_Init(); ReadLine(&error);
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

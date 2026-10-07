/* 通用位置式 PID。积分变量保存 Ki*误差*时间累加后的输出贡献，不是裸误差积分。
 * 微分低通和条件积分防止噪声放大、输出饱和后积分越积越多。
 */
#include "PID.h"
static float Clamp(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}
/* 清零记忆，上一次误差不影响新阶段。 */
void PID_Reset(PID_TypeDef *pid)
{
    pid->target = pid->actual = pid->error = pid->last_error = 0.0f;
    pid->integral = pid->derivative = pid->output = 0.0f;
    pid->ready = 0;
}
/* 保存增益、积分上限和微分滤波时间，再复位。 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float integral_limit, float derivative_tau)
{
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->integral_limit = integral_limit;
    pid->derivative_tau = derivative_tau;
    PID_Reset(pid);
}
/* 每次推进一个控制周期。dt 单位秒，feedforward 是预估的基本驱动力。 */
float PID_Step(PID_TypeDef *pid, float error, float dt,
               float feedforward, float lower, float upper)
{
    float raw_d, candidate_i, candidate_out, base;
    if (dt <= 0.0f || upper < lower) return pid->output;
    /* 第一帧无历史误差，微分设为零，避免起步冲击。 */
    raw_d = pid->ready ? (error - pid->last_error) / dt : 0.0f;
    pid->derivative += dt / (pid->derivative_tau + dt) *
                       (raw_d - pid->derivative);
    pid->ready = 1;
    pid->error = pid->last_error = error;
    base = feedforward + pid->Kp * error + pid->Kd * pid->derivative;
    candidate_i = Clamp(pid->integral + pid->Ki * error * dt,
                        -pid->integral_limit, pid->integral_limit);
    candidate_out = base + candidate_i;
    /* 输出已顶到上限时不继续正向积分；误差反向时允许积分退回来。 */
    if (!((candidate_out > upper && error > 0.0f) ||
          (candidate_out < lower && error < 0.0f)))
        pid->integral = candidate_i;
    pid->output = Clamp(base + pid->integral, lower, upper);
    return pid->output;
}

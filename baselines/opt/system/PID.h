#ifndef __PID_H
#define __PID_H
typedef struct {
    float Kp, Ki, Kd;
    float target, actual, error, last_error;
    float integral, derivative, output;
    float integral_limit, derivative_tau;
    unsigned char ready;
} PID_TypeDef;
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float integral_limit, float derivative_tau);
void PID_Reset(PID_TypeDef *pid);
/* dt 单位为秒；integral 是对最终输出的积分贡献。 */
float PID_Step(PID_TypeDef *pid, float error, float dt,
               float feedforward, float lower, float upper);
#endif

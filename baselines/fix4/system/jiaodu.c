#include "jiaodu.h"
#include "jy901s.h"
#include "Motor.h"
#include "math.h"
#include "Delay.h"
#include "OLED.h"

// ===================== 转向PID参数 =====================
// 原地转向专用（位置式PID，带积分）
float steer_kp = 2.4f;
float steer_ki = 0.0f;
float steer_kd = 0.7f;

// 直线行驶专用（PD控制，无积分，解决越偏越大）
float straight_kp = 2.2f;
float straight_kd = 0.6f;

/**
  * @brief  角度误差限幅函数（处理360度循环问题）
  * @param  err 原始角度误差
  * @retval 限幅后的角度误差（范围 -180 ~ 180）
  */
float Angle_Err_Limit(float err)
{
    while(err > 180)  err -= 360;
    while(err < -180) err += 360;
    return err;
}

/**
  * @brief  原地转向专用：位置式PID（带积分）
  */
float PID_Turn_Only(float target, float measure)
{
    static float curr_err  = 0.0f;
    static float last_err1 = 0.0f;
    static float integral  = 0.0f;

    curr_err = Angle_Err_Limit(target - measure);

    // 积分累加 + 限幅
    integral += curr_err;
    if(integral > 12)  integral = 12;
    if(integral < -12) integral = -12;

    float pid_out = steer_kp * curr_err
                  + steer_ki * integral
                  + steer_kd * (curr_err - last_err1);

    last_err1 = curr_err;
    return pid_out;
}

/**
  * @brief  直线纠偏专用：PD控制（无积分，彻底解决越偏越大）
  */
float PID_Straight_PD(float target, float measure)
{
    static float last_err = 0.0f;

    float curr_err = Angle_Err_Limit(target - measure);
    float pid_out = straight_kp * curr_err + straight_kd * (curr_err - last_err);

    last_err = curr_err;
    return pid_out;
}

// 外部全局变量
extern volatile float global_angle;
extern volatile uint8_t new_data_received;

/*********************************************************
函数名：Gyro_Turn_To_Angle
功能：陀螺仪控制车体旋转到指定角度（原地旋转，位置式PID）
*********************************************************/
void Gyro_Turn_To_Angle(float target_angle)
{
	float current_angle;
	float pid_out;
	int16_t turn_pwm;

	#define MAX_SPEED    30
	#define TOLERANCE    1.5f
	#define LOOP_DELAY   10

	while(1)
	{
		if(new_data_received == 0)
		{
			Delay_ms(LOOP_DELAY);
			continue;
		}

		__disable_irq();
		current_angle = global_angle;
		new_data_received = 0;
		__enable_irq();

		// 转向用位置式PID
		pid_out = PID_Turn_Only(target_angle, current_angle);

		float error = Angle_Err_Limit(target_angle - current_angle);
		if(fabs(error) < TOLERANCE)
		{
			Motor_SetLeftspeed(0);
			Motor_SetRightspeed(0);
			break;
		}

		turn_pwm = (int16_t)pid_out;
		if(turn_pwm >  MAX_SPEED) turn_pwm =  MAX_SPEED;
		if(turn_pwm < -MAX_SPEED) turn_pwm = -MAX_SPEED;

		Motor_SetLeftspeed( turn_pwm);
		Motor_SetRightspeed(-turn_pwm);

		Delay_ms(LOOP_DELAY);
	}
}

/*********************************************************
函数名：Gyro_Go_Straight_With_Angle
功能：陀螺仪控制车体保持指定角度直线行驶（PD纠偏，不飘角）
*********************************************************/
void Gyro_Go_Straight_With_Angle(float target_angle, int16_t base_speed)
{
    #define MAX_CORRECT  14
    #define MAX_SPEED    30

    float current_angle;

    __disable_irq();
    current_angle = global_angle;
    __enable_irq();

    OLED_ShowSignedNum(1, 9, (int)current_angle, 4);

    // 直线用PD控制（无积分，不会越偏越大）
    float pid_out = PID_Straight_PD(target_angle, current_angle);

    int16_t correct = (int16_t)pid_out;
    if(correct > MAX_CORRECT) correct = MAX_CORRECT;
    if(correct < -MAX_CORRECT) correct = -MAX_CORRECT;

    int16_t left_speed  = base_speed + correct;
    int16_t right_speed = base_speed - correct;

    // 速度限幅
    if(left_speed >  MAX_SPEED) left_speed =  MAX_SPEED;
    if(left_speed < -MAX_SPEED) left_speed = -MAX_SPEED;
    if(right_speed >  MAX_SPEED) right_speed =  MAX_SPEED;
    if(right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

    Motor_SetLeftspeed(left_speed);
    Motor_SetRightspeed(right_speed);
}

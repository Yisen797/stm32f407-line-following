#include "pid.h"
#include <math.h>  /* fabsf() */

void pid_init(PID *pid, float kp, float ki, float kd, float dt,
              float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->integral_max =  integral_limit;
    pid->integral_min = -integral_limit;
    pid->prev_error = 0.0f;
    pid->output_max =  output_limit;
    pid->output_min = -output_limit;
}

float pid_calc(PID *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;      /* 误差 = 目标 - 实际 */

    /* --- 比例项 --- */
    float p_term = pid->kp * error;

    /* --- 积分项 (带 anti-windup) --- */
    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < pid->integral_min)
        pid->integral = pid->integral_min;
    float i_term = pid->ki * pid->integral;

    /* --- 微分项 (微分先行: 对 measurement 微分，减少阶跃冲击) --- */
    float d_term = pid->kd * (pid->prev_error - error) / pid->dt;
    pid->prev_error = error;

    /* --- 合成输出并限幅 --- */
    float output = p_term + i_term + d_term;
    if (output > pid->output_max)  output = pid->output_max;
    if (output < pid->output_min)  output = pid->output_min;

    return output;
}

void pid_reset(PID *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set_gains(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

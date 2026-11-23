#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/**
 * PID 控制器 — 通用位置式 PID
 * =============================
 *
 * 公式:
 *   output = Kp*err + Ki*∫err*dt + Kd*(err - prev_err)/dt
 *
 * 内置积分限幅 (anti-windup)，防止积分饱和。
 */

typedef struct {
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */
    float dt;           /* 控制周期 (秒) */

    float integral;     /* 积分累加值 */
    float integral_max; /* 积分上限 */
    float integral_min; /* 积分下限 */
    float prev_error;   /* 上次误差 (用于微分) */

    float output_min;   /* 输出下限 */
    float output_max;   /* 输出上限 */
} PID;

/** 初始化 PID 结构体 */
void pid_init(PID *pid, float kp, float ki, float kd, float dt,
              float integral_limit, float output_limit);

/** 计算一次 PID 输出 */
float pid_calc(PID *pid, float setpoint, float measurement);

/** 复位 PID 内部状态 */
void pid_reset(PID *pid);

/** 在线整定: 修改增益 */
void pid_set_gains(PID *pid, float kp, float ki, float kd);

#endif

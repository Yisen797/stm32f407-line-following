#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/**
 * TB6612 双路电机驱动模块
 * ========================
 *
 * 引脚连接:
 *   PWMA  → PA7 (TIM3_CH2)    左电机 PWM
 *   PWMB  → PA6 (TIM3_CH1)    右电机 PWM
 *   AIN1  → PC0  左电机方向1
 *   AIN2  → PC1  左电机方向2
 *   BIN1  → PC2  右电机方向1
 *   BIN2  → PC3  右电机方向2
 *
 * 方向控制:
 *   AIN1=1,AIN2=0 → 正转    AIN1=0,AIN2=1 → 反转
 *   AIN1=0,AIN2=0 → 惰行    AIN1=1,AIN2=1 → 刹车
 *
 * 速度范围: -1000 ~ 1000 (负数 = 反转, 正数 = 正转)
 *   PWM 分辨率: 1000 级 (ARR=999)
 *   PWM 频率:   约 8kHz @ 8MHz TIM3 时钟
 */

/** 初始化 TIM3 PWM 和 TB6612 方向控制 GPIO */
void motor_init(void);

/**
 * 设置左电机速度
 * @param speed  -1000 ~ 1000, 负值=反转, 正值=正转
 */
void motor_left_set(int16_t speed);

/**
 * 设置右电机速度
 * @param speed  -1000 ~ 1000, 负值=反转, 正值=正转
 */
void motor_right_set(int16_t speed);

/** 两轮同时刹车 (AIN1=AIN2=1) */
void motor_brake(void);

/** 两轮惰行 (AIN1=AIN2=0) */
void motor_coast(void);

#endif

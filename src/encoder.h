#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/**
 * 霍尔编码器 — 双电机测速模块
 * ==============================
 *
 * 左电机: TIM2 编码器模式  CH1=PA0, CH2=PA1
 * 右电机: TIM8 编码器模式  CH1=PC6, CH2=PC7
 *
 * 编码器模式原理:
 *   定时器的 CH1/CH2 接到编码器 A/B 相。
 *   电机正转时 A 超前 B 90°, 计数器递增。
 *   反转时 B 超前 A 90°, 计数器递减。
 *   TIM2_CR1 的 DIR 位指示方向。
 */

/** 初始化 TIM2 和 TIM9 编码器接口 */
void encoder_init(void);

/**
 * 读取编码器原始计数值
 * 注意: 读取后计数器不会自动清零。
 *       在采样周期内两次读数之差 = 脉冲增量。
 *       TIM9 为 16-bit, 需在 ~10ms 内采样否则可能溢出。
 */
int32_t encoder_read_left(void);
int16_t encoder_read_right(void);

/** 获取速度 (脉冲数/秒), dt=距上次调用间隔(秒) */
int32_t encoder_speed_left(float dt);
int32_t encoder_speed_right(float dt);

/** 清零编码器计数器 */
void encoder_reset(void);

#endif

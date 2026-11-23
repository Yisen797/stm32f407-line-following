#ifndef __TRACKER_H
#define __TRACKER_H

#include <stdint.h>

/**
 * 三轮寻迹小车控制逻辑 — 核心算法模块
 * =====================================
 *
 * 硬件组合:
 *   8 路红外传感器 (PD0~PD7) → 计算黑线位置
 *   PID 控制器 → 转向修正量
 *   TB6612 + MG370 → 左右电机差速驱动
 *
 * 控制策略 (平稳优先):
 *   1. 传感器阵列 → 加权平均求黑线中心位置 → 误差
 *   2. 位置 PID → 转向修正
 *   3. base_speed ± correction → 左右轮目标速度
 *   4. 输出 PWM 到电机驱动板
 *
 * 传感器位置映射:
 *   X1 (PD0, 最左)  → 位置坐标 0
 *   X2 (PD1)        → 位置坐标 1
 *   ...
 *   X8 (PD7, 最右)  → 位置坐标 7
 *   轨道中心 ⇒ 位置坐标 3.5
 *   误差 = 当前位置 - 3.5
 */

/** 初始化追踪器状态 (必须在 motor_init / sensor_init 之后调用) */
void tracker_init(void);

/**
 * 执行一次寻迹控制循环
 *
 * 每 dt 秒调用一次 (建议 10~20ms):
 *   1. 读取传感器
 *   2. 计算误差
 *   3. PID 计算修正量
 *   4. 更新左右电机 PWM
 *
 * @param dt        控制周期 (秒), 典型值 0.01~0.02
 * @param base_speed 基准速度 (0~1000), 越大越快
 * @return  0=正常, >0=检测到黑线数, <0=完全丢线
 */
int tracker_update(float dt, int16_t base_speed);

/** 紧急停车 (两轮刹车) */
void tracker_stop(void);

/** 获取当前黑线偏离量 (用于 OLED 显示) */
float tracker_get_error(void);

/** 在线调整 PID 参数 */
void tracker_tune_pid(float kp, float ki, float kd);

#endif

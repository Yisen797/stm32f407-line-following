#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

/**
 * HC-SR04 超声波测距模块
 * Trig = PF5 (输出), Echo = PF6 (输入)
 * 距离范围: 2cm ~ 400cm
 */

void ultrasonic_init(void);

/** 测距一次, 返回距离 (cm), 超时返回 -1 */
float ultrasonic_read(void);

#endif

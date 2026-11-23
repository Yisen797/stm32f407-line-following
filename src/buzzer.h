#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>

/**
 * 蜂鸣器模块 — PF3，低电平触发
 *
 * 用法:
 *   buzzer_init();      // 初始化 PF3 为推挽输出
 *   buzzer_on();        // 鸣响
 *   buzzer_off();       // 关闭
 *   buzzer_beep(ms);    // 鸣响指定毫秒后自动关闭
 */

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(uint32_t ms);

#endif

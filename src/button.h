#ifndef __BUTTON_H
#define __BUTTON_H

#include <stdint.h>

/** KEY 按键 (PA15, 上拉, 按下接GND=低电平) */
void button_init(void);

/** 检测是否按下 (读取 PD13 电平) */
uint8_t button_pressed(void);

#endif

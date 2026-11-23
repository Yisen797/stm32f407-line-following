#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

/**
 * 软件毫秒级延时
 *
 * 为什么不用 HAL_Delay()?
 *   HAL_Delay() 依赖 SysTick 中断来计时。在本工程的 PlatformIO
 *   + STM32Cube HAL 环境下，SysTick 中断不触发，导致 HAL_Delay()
 *   永远卡死。所以我们自己用空循环实现延时。
 *
 * 精度说明:
 *   基于 HSI 16MHz 内部时钟，每个循环约 4000 次空操作 ≈ 1ms。
 *   不是高精度延时，但对传感器轮询、LED 闪烁、OLED 等待足够了。
 */

void delay_ms(uint32_t ms);

#endif

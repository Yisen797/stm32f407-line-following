#include "delay.h"

/**
 * 软件毫秒级延时（空循环实现）
 *
 * @param ms  延时毫秒数
 *
 * 原理:
 *   用两层 for 循环消耗 CPU 时间来近似 1ms 延时。
 *   外层循环 ms 次，内层约 4000 次循环 ≈ 1ms。
 *
 *   使用 volatile 修饰循环变量，防止编译器优化掉循环体。
 *
 * 校准:
 *   4000 这个值是在 HSI 16MHz、无 PLL 的条件下通过
 *   LED 闪烁比对校准的，误差在可接受范围内。
 */
void delay_ms(uint32_t ms)
{
    while (ms--) {
        /* HSE 168MHz 下约 42000 次循环 ≈ 1ms */
        for (volatile uint32_t i = 0; i < 42000; i++) {
        }
    }
}

#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>

/**
 * 8 路红外寻迹传感器模块
 * =======================
 *
 * 硬件:
 *   亚博智能 8 路红外寻迹模块
 *   输出: 数字信号, 1 = 检测到黑线, 0 = 白色/无线
 *
 * 引脚连接 (F407 核心板):
 *   X1 → PD0   X2 → PD1   X3 → PD2   X4 → PD3
 *   X5 → PD4   X6 → PD5   X7 → PD6   X8 → PD7
 *
 * 供电: VCC → 3.3V (或 5V, 视模块而定)
 *        GND → GND
 */

/** 初始化 PD0-PD7 为数字输入（带下拉电阻） */
void sensor_init(void);

/**
 * 读取 8 路传感器当前状态
 *
 * @return 8-bit 值:
 *         bit0 (LSB) = PD0 = X1 (最左)
 *         bit1       = PD1 = X2
 *         ...
 *         bit7 (MSB) = PD7 = X8 (最右)
 *
 *         每个 bit: 1 = 检测到黑线, 0 = 无线/白色
 */
uint8_t sensor_read(void);

#endif

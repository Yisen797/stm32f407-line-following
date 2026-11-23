#ifndef __OLED_SSD1306_H
#define __OLED_SSD1306_H

#include <stdint.h>

/**
 * SSD1306 OLED 驱动模块
 * ======================
 *
 * 硬件参数:
 *   尺寸:   0.96 英寸
 *   分辨率: 128 x 64 像素
 *   颜色:   蓝白单色（蓝底白字）
 *   接口:   I2C
 *   地址:   7-bit 0x3C
 *
 * 引脚连接:
 *   SCL -> PB8  (I2C1 时钟)
 *   SDA -> PB9  (I2C1 数据)
 *   VCC -> 3.3V
 *   GND -> GND
 *
 * 驱动芯片内部结构:
 *   屏幕分为 8 个"页"(page)，每页 8 像素高，128 像素宽。
 *   每页 = 128 字节，1 字节 = 纵向 8 个像素（LSB 在上）。
 *
 *   页 0 ┌────────────── 128 列 ──────────────┐
 *   页 1 ├──────────────────────────────────────┤
 *   ...  │                                      │
 *   页 7 └──────────────────────────────────────┘
 */

/** 初始化 I2C1 总线和 SSD1306 屏幕 */
void oled_init(void);

/** 清空屏幕（所有像素熄灭） */
void oled_clear(void);

/** 显示棋盘格图案（快速视觉测试，确认 OLED 工作正常） */
void oled_chess(void);

/**
 * 在指定位置显示字符串
 *
 * @param col   起始列 (像素, 0-121, 每个字符 6 像素宽)
 * @param page  起始页 (0-7, 对应 8 行文字)
 * @param str   要显示的 ASCII 字符串 (支持 32-94 号字符)
 */
void oled_text(uint8_t col, uint8_t page, const char *str);

#endif

#include "sensor.h"
#include "stm32f4xx_hal.h"

/**
 * 初始化 8 路传感器 GPIO
 *
 * 配置 GPIOD 的 0-7 号引脚为数字输入:
 *   - Mode = Input (输入模式, 读取外部电平)
 *   - Pull = Pull-down (下拉, 悬空时默认为 0)
 *
 * 为什么用下拉?
 *   传感器输出高电平 = 1 (检测到黑线)。
 *   刚上电或传感器松动时，引脚悬空 → 下拉电阻将电平拉低 → 读到 0，
 *   避免悬空时读到随机值。
 */
void sensor_init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();  /* 开启 GPIOD 时钟 */

    GPIO_InitTypeDef cfg = {0};
    cfg.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
             | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    cfg.Mode = GPIO_MODE_INPUT;      /* 数字输入模式 */
    cfg.Pull = GPIO_PULLDOWN;        /* 下拉: 无信号时读 0 */

    HAL_GPIO_Init(GPIOD, &cfg);
}

/**
 * 读取 8 路传感器
 *
 * 直接读 GPIOD 的输入数据寄存器 (IDR) 的低 8 位。
 * IDR 的 bit0-bit7 对应 PD0-PD7 的电平状态。
 * 一次读 32 位，取低 8 位即可。
 */
uint8_t sensor_read(void)
{
    return (uint8_t)(GPIOD->IDR & 0xFF);
}

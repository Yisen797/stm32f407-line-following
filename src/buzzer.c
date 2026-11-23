#include "buzzer.h"
#include "delay.h"
#include "stm32f4xx_hal.h"

/**
 * 蜂鸣器 — PF3，低电平触发
 * PF3 在 F407 上是普通 GPIO，直接配为输出即可。
 */

void buzzer_init(void)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_3;
    g.Mode  = GPIO_MODE_OUTPUT_PP;   /* 推挽输出 */
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOF, &g);
    buzzer_off();  /* 初始静音 */
}

void buzzer_on(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);  /* 低电平 → 鸣响 */
}

void buzzer_off(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);    /* 高电平 → 静音 */
}

void buzzer_beep(uint32_t ms)
{
    buzzer_on();
    delay_ms(ms);
    buzzer_off();
}

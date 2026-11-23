#include "ultrasonic.h"
#include "stm32f4xx_hal.h"
#include "delay.h"

/**
 * HC-SR04 超声波测距
 * Trig = PF5, Echo = PF6
 * TIM5 用作 1MHz 自由计数器 (1 tick = 1µs)
 * 距离 = 脉宽(µs) / 58 (cm)
 */

#define TRIG_PIN  GPIO_PIN_5
#define ECHO_PIN  GPIO_PIN_6

void ultrasonic_init(void)
{
    /* GPIOF 时钟 */
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* PF5 = 推挽输出 (Trig) */
    GPIO_InitTypeDef g = {0};
    g.Pin   = TRIG_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &g);
    HAL_GPIO_WritePin(GPIOF, TRIG_PIN, GPIO_PIN_RESET);

    /* PF6 = 输入 (Echo) */
    g.Pin  = ECHO_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &g);

    /* TIM5: 1MHz 自由计数器
     * APB1=42MHz, 定时器时钟=APB1×2=84MHz
     * PSC=83 → 84MHz/84 = 1MHz (1µs/tick) */
    __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->PSC = 83;
    TIM5->ARR = 0xFFFFFFFF;
    TIM5->CNT = 0;
    TIM5->CR1 = TIM_CR1_CEN;
}

float ultrasonic_read(void)
{
    uint32_t t_start, t_end;

    /* 发送 15µs 触发脉冲 (HC-SR04 要求 >=10µs) */
    HAL_GPIO_WritePin(GPIOF, TRIG_PIN, GPIO_PIN_SET);
    {
        uint32_t t0 = TIM5->CNT;
        while ((TIM5->CNT - t0) < 15);  /* 15µs @ 1MHz */
    }
    HAL_GPIO_WritePin(GPIOF, TRIG_PIN, GPIO_PIN_RESET);

    /* 等待 Echo 上升沿 (TIM5 超时 38ms = 38000µs, 对应 ~6.5m) */
    uint32_t t0 = TIM5->CNT;
    while (!(GPIOF->IDR & ECHO_PIN)) {
        if ((TIM5->CNT - t0) > 38000) return -1.0f;
    }
    t_start = TIM5->CNT;

    /* 等待 Echo 下降沿 */
    while (GPIOF->IDR & ECHO_PIN) {
        if ((TIM5->CNT - t_start) > 38000) return -1.0f;
    }
    t_end = TIM5->CNT;

    /* 脉宽 (µs) → 距离 (cm) = 脉宽 / 58 */
    uint32_t pulse = (t_end >= t_start) ? (t_end - t_start)
                     : (0xFFFFFFFF - t_start + t_end + 1);
    return (float)pulse / 58.0f;
}

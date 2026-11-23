#include "encoder.h"
#include "stm32f4xx_hal.h"

/**
 * 编码器初始化
 *
 * TIM2 (32-bit, APB1): 左电机编码器 PA0=CH1, PA1=CH2, AF=1
 * TIM8 (16-bit, APB2): 右电机编码器 PC6=CH1, PC7=CH2, AF=3
 */
void encoder_init(void)
{
    /* --- GPIO 时钟 --- */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* --- TIM2 时钟 (APB1) --- */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* --- TIM8 时钟 (APB2) --- */
    __HAL_RCC_TIM8_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;

    /* 左编码器: PA0+PA1 → TIM2 */
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 右编码器: PC6+PC7 → TIM8 */
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* ================================================================
     * TIM2 编码器模式 (左电机)
     *
     * 编码器模式 3: 两个边沿都计数 → 4 倍频
     *   即每个编码器脉冲产生 4 个计数。
     *   例如 11 PPR 电机转一圈 → 44 计数。
     *
     * SMCR: SMS = 011 (Encoder mode 3)
     * CCMR1: CC1S=01 (TI1), CC2S=01 (TI2)
     * ================================================================ */
    TIM2->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos)   /* CC1 → TI1 */
                | (1 << TIM_CCMR1_CC2S_Pos);  /* CC2 → TI2 */
    TIM2->SMCR  = 3;  /* SMS = 011: Encoder mode 3 (TI1+TI2 双沿) */
    TIM2->ARR   = 0xFFFFFFFF;  /* 32-bit 最大计数 */
    TIM2->CNT   = 0;
    TIM2->CR1  |= TIM_CR1_CEN;  /* 使能计数器 */

    /* ================================================================
     * TIM8 编码器模式 (右电机)
     * ================================================================ */
    TIM8->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos)
                | (1 << TIM_CCMR1_CC2S_Pos);
    TIM8->SMCR  = 3;
    TIM8->ARR   = 0xFFFF;
    TIM8->CNT   = 0;
    TIM8->CR1  |= TIM_CR1_CEN;
}

int32_t encoder_read_left(void)
{
    return (int32_t)TIM2->CNT;
}

int16_t encoder_read_right(void)
{
    return (int16_t)TIM9->CNT;
}

int32_t encoder_speed_left(float dt)
{
    static int32_t last = 0;
    int32_t now = (int32_t)TIM2->CNT;
    int32_t delta = now - last;
    last = now;
    return (int32_t)((float)delta / dt);
}

int32_t encoder_speed_right(float dt)
{
    static int32_t last = 0;
    int32_t now = (int32_t)TIM8->CNT;
    int32_t delta = now - last;
    last = now;
    return -(int32_t)((float)delta / dt);  /* 方向修正 */
}

void encoder_reset(void)
{
    TIM2->CNT = 0;
    TIM8->CNT = 0;
}

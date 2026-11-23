#include "motor.h"
#include "stm32f4xx_hal.h"

/* ARR = 999 → PWM 分辨率 = 1000 (0~999), 频率 ≈ 8kHz */
#define PWM_ARR  999

/* 方向引脚位定义 */
#define AIN1_PIN  GPIO_PIN_0   /* PC0 */
#define AIN2_PIN  GPIO_PIN_1   /* PC1 */
#define BIN1_PIN  GPIO_PIN_2   /* PC2 */
#define BIN2_PIN  GPIO_PIN_3   /* PC3 */

void motor_init(void)
{
    /* ================================================================
     * 第 1 步: GPIO 初始化 (PC0~PC3 = 方向控制)
     * ================================================================ */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;    /* 推挽输出 */
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOC, &g);

    /* 初始状态: 惰行 (方向引脚全低) */
    GPIOC->BSRR = (AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN) << 16;

    /* ================================================================
     * 第 2 步: TIM3 PWM 初始化 (PA6=CH1, PA7=CH2)
     *
     * TIM3 在 APB1 上，时钟 = 8MHz (HSI 16MHz, APB1/4)
     * PWM 频率 = 8MHz / (ARR+1) = 8MHz / 1000 ≈ 8kHz
     * PWM 模式 1: CNT < CCR → 高电平, CNT >= CCR → 低电平
     * ================================================================ */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PA6 + PA7 → 复用功能 AF2 (TIM3) */
    GPIO_InitTypeDef pwm_gpio = {0};
    pwm_gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    pwm_gpio.Mode      = GPIO_MODE_AF_PP;
    pwm_gpio.Pull      = GPIO_NOPULL;
    pwm_gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    pwm_gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &pwm_gpio);

    /* TIM3 时基 */
    TIM3->PSC = 0;           /* 不分频 */
    TIM3->ARR = PWM_ARR;     /* 自动重装载 = 999 */
    TIM3->CNT = 0;

    /* CH1 (PA6): PWM 模式 1, 初始占空比 0% */
    TIM3->CCMR1  = (6 << TIM_CCMR1_OC1M_Pos)   /* OC1M = PWM mode 1 */
                 | (1 << TIM_CCMR1_OC1PE_Pos); /* 输出预装载使能 */
    TIM3->CCER  |= TIM_CCER_CC1E;               /* CH1 输出使能 */
    TIM3->CCR1   = 0;                           /* 占空比 = 0 */

    /* CH2 (PA7): 同上 */
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);
    TIM3->CCER  |= TIM_CCER_CC2E;
    TIM3->CCR2   = 0;

    /* 启动 TIM3 */
    TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;  /* ARPE = ARR 预装载 */
}

/* ================================================================
 * 设置电机: 带符号速度 → 方向 + PWM 占空比
 *
 * TB6612 控制逻辑:
 *   speed > 0: 正转 (AIN1=1, AIN2=0), PWM = speed
 *   speed < 0: 反转 (AIN1=0, AIN2=1), PWM = |speed|
 *   speed = 0: 惰行 (AIN1=0, AIN2=0)
 *
 * 注意: 左电机 BO1(BO2) 接到 A 通道; 右电机 AO1(AO2) 接到 B 通道。
 *       左用 AIN1/AIN2, 右用 BIN1/BIN2。
 * ================================================================ */

void motor_left_set(int16_t speed)
{
    uint16_t pwm;
    uint32_t bsrr = 0;

    if (speed > 0) {
        bsrr = BIN2_PIN | (BIN1_PIN << 16);         /* BIN2=1, BIN1=0 → 正转 (方向修正) */
        pwm = (speed > PWM_ARR) ? PWM_ARR : (uint16_t)speed;
    } else if (speed < 0) {
        bsrr = BIN1_PIN | (BIN2_PIN << 16);         /* BIN1=1, BIN2=0 → 反转 */
        pwm = (speed < -PWM_ARR) ? PWM_ARR : (uint16_t)(-speed);
    } else {
        bsrr = (BIN1_PIN | BIN2_PIN) << 16;          /* BIN1=0, BIN2=0 → 惰行 */
        pwm = 0;
    }

    GPIOC->BSRR = bsrr;
    TIM3->CCR1  = pwm;   /* CH1 = PA6 = 左电机 (B 通道) */
}

void motor_right_set(int16_t speed)
{
    uint16_t pwm;
    uint32_t bsrr = 0;

    if (speed > 0) {
        bsrr = AIN1_PIN | (AIN2_PIN << 16);         /* AIN1=1, AIN2=0 → 正转 */
        pwm = (speed > PWM_ARR) ? PWM_ARR : (uint16_t)speed;
    } else if (speed < 0) {
        bsrr = AIN2_PIN | (AIN1_PIN << 16);         /* AIN2=1, AIN1=0 → 反转 */
        pwm = (speed < -PWM_ARR) ? PWM_ARR : (uint16_t)(-speed);
    } else {
        bsrr = (AIN1_PIN | AIN2_PIN) << 16;          /* AIN1=0, AIN2=0 → 惰行 */
        pwm = 0;
    }

    GPIOC->BSRR = bsrr;
    TIM3->CCR2  = pwm;   /* CH2 = PA7 = 右电机 (A 通道) */
}

void motor_brake(void)
{
    /* AIN1=AIN2=1, BIN1=BIN2=1 → 短路刹车 */
    GPIOC->BSRR = AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
}

void motor_coast(void)
{
    /* 所有方向引脚拉低 → 惰行 */
    uint32_t mask = AIN1_PIN | AIN2_PIN | BIN1_PIN | BIN2_PIN;
    GPIOC->BSRR = mask << 16;  /* BRR: 清 0 */
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
}

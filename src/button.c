#include "button.h"
#include "stm32f4xx_hal.h"

void button_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_15;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;  /* 默认高电平, 按下接GND变低 */
    HAL_GPIO_Init(GPIOA, &g);
}

uint8_t button_pressed(void)
{
    return (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15));
}

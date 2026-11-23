/**
 * 三轮寻迹小车 — STM32F407ZGT6  HSE 168MHz
 *
 * 1.54" LCD: 系统状态 + IR数据 + 偏离 + 方向
 * 0.96" OLED: 状态 + 超声波距离
 * 按键 KEY(PA15): 启动/停止, 电机延迟2s启动
 */

#include "stm32f4xx_hal.h"
#include "delay.h"
#include "oled_ssd1306.h"
#include "lcd_spi_154.h"
#include "sensor.h"
#include "ultrasonic.h"
#include "motor.h"
#include "encoder.h"
#include "tracker.h"
#include "button.h"
#include <stdio.h>

#define LED_ON()   (GPIOC->BSRR = (1U << (13+16)))
#define LED_OFF()  (GPIOC->BSRR = (1U << 13))

/* 编码器参数 (需实测校准) */
#define PULSES_PER_REV  390    /* 编码器线数 × 减速比 (电机一转脉冲数) */
#define WHEEL_DIA_MM    65     /* 轮胎直径 mm */
#define WHEEL_CIRC_MM   204.2f /* 轮胎周长 = π×65mm */

static void clock_hse_168mhz(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; PWR->CR |= PWR_CR_VOS;
    while (!(PWR->CSR & PWR_CSR_VOSRDY));
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN
               | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    RCC->CR |= RCC_CR_HSEON;
    /* 等待 HSE 就绪 — RST 后晶振重启需要更长时间 */
    for (volatile int w = 0; w < 2000000; w++) {
        if (RCC->CR & RCC_CR_HSERDY) break;
    }
    if (!(RCC->CR & RCC_CR_HSERDY)) {
        /* HSE 失败, 使用 HSI */
        SystemCoreClock = 16000000;
        return;
    }
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE
                 | (8 << RCC_PLLCFGR_PLLM_Pos) | (336 << RCC_PLLCFGR_PLLN_Pos)
                 | (0 << RCC_PLLCFGR_PLLP_Pos) | (7 << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
              | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    SystemCoreClock = 168000000;
}

int main(void)
{
    clock_hse_168mhz();

    oled_init();        SPI_LCD_Init();  /* OLED 必须在 LCD 之前初始化 */
    button_init();     sensor_init();
    /* ultrasonic_init(); */ motor_init();  /* 超声波硬件未接 */
    motor_brake();     encoder_init();   tracker_init();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIOC->MODER &= ~(3U<<26); GPIOC->MODER |= (1U<<26);
    for(int i=0;i<3;i++){LED_ON();delay_ms(80);LED_OFF();delay_ms(80);}

    /* ===== 1.54" LCD 启动画面 ===== */
    LCD_SetDirection(Direction_H);
    LCD_SetBackColor(LCD_BLACK); LCD_Clear();
    LCD_SetAsciiFont(&ASCII_Font24);
    LCD_SetColor(LCD_WHITE);
    LCD_DisplayString(30, 60, "TRACKER READY");
    LCD_SetAsciiFont(&ASCII_Font20);
    LCD_SetColor(LCD_CYAN);
    LCD_DisplayString(36, 100, "Press KEY");
    LCD_SetAsciiFont(&ASCII_Font16);
    LCD_SetColor(LCD_GREY);
    LCD_DisplayString(60, 140, "HSE 168MHz");

    /* ===== OLED ===== */
    oled_chess(); delay_ms(150); oled_clear();
    oled_text(0, 3, "  Motor: STOP");

    /* ===== 状态机 ===== */
    int running = 0, btn_prev = 0;
    const int16_t base_speed = 180;  /* 低速平稳优先 */
    const float   dt = 0.02f;
    uint32_t lost_ticks = 0, motor_timer = 0;
    int line_cnt = 0, motors_on = 0;

    while (1)
    {
        /* 按键 */
        int btn_now = button_pressed();
        if (btn_now) LED_ON(); else LED_OFF();

        if (btn_now && !btn_prev) {
            running = !running;
            if (running) {
                tracker_init(); lost_ticks = 0;
                motors_on = 0;  motor_timer = 0;
                LCD_SetBackColor(LCD_BLACK); LCD_Clear();
                LCD_SetAsciiFont(&ASCII_Font24);
                LCD_SetColor(LCD_GREEN);
                LCD_DisplayString(24, 20, "TRACKER RUNNING");
                /* 绘制静态布局 (只画一次) */
                LCD_SetAsciiFont(&ASCII_Font20); LCD_SetColor(LCD_WHITE);
                LCD_DisplayString(10, 52, "IR:");
                LCD_DisplayString(10, 82, "Dev:");
                LCD_SetColor(LCD_GREY); LCD_FillRect(20, 150, 200, 8);
                LCD_SetColor(LCD_YELLOW);
                LCD_SetAsciiFont(&ASCII_Font20);
                LCD_DisplayString(36, 170, "Motor wait 2s..");
                LCD_SetAsciiFont(&ASCII_Font16);
                oled_clear();
                oled_text(0, 1, "--- Wheel Speed ---");
            } else {
                tracker_stop(); motors_on = 0;
                LCD_SetBackColor(LCD_BLACK); LCD_Clear();
                LCD_SetAsciiFont(&ASCII_Font24);
                LCD_SetColor(LCD_WHITE);
                LCD_DisplayString(30, 60, "TRACKER READY");
                LCD_SetAsciiFont(&ASCII_Font20);
                LCD_SetColor(LCD_CYAN);
                LCD_DisplayString(48, 100, "Press KEY");
                oled_clear();
                oled_text(0, 3, "  Motor: STOP");
            }
            delay_ms(200);
        }
        btn_prev = btn_now;

        /* 控制 */
        if (running) {
            delay_ms(20);
            if (motors_on)
                line_cnt = tracker_update(dt, base_speed);
            else
                line_cnt = 0;
        } else {
            delay_ms(50);
        }

        /* 电机延迟启动 */
        if (running && !motors_on) {
            motor_timer++;
            if (motor_timer > 50) {  /* 50*20ms = 1s */
                motors_on = 1;
                LCD_SetAsciiFont(&ASCII_Font20);
                LCD_SetColor(LCD_GREEN);
                LCD_DisplayString(40, 180, "MOTOR ON");
            }
        }

        uint8_t raw = sensor_read();
        if (running && line_cnt <= 0) lost_ticks++;
        else lost_ticks = 0;
        if (lost_ticks > 25) { tracker_stop(); lost_ticks = 0; }

        /* OLED 刷新 — 电机相关 (每 200ms) */
        static int dc = 0;
        if (++dc >= 10) {
            dc = 0;
            char buf[21];
            if (running) {
                int32_t spd_l = (int32_t)((float)encoder_speed_left(0.2f)
                                / PULSES_PER_REV * WHEEL_CIRC_MM);
                int32_t spd_r = (int32_t)((float)encoder_speed_right(0.2f)
                                / PULSES_PER_REV * WHEEL_CIRC_MM);
                snprintf(buf,sizeof(buf),"L:%+4ld R:%+4ld mm/s",spd_l,spd_r);
                oled_text(0, 2, buf);
                float dist = -1; /* ultrasonic_read() — 硬件未接 */
                if (dist > 0) { int cm=(int)dist,mm=(int)((dist-(float)cm)*10.0f);
                    snprintf(buf,sizeof(buf),"D:%2d.%d MTR:%s",cm,mm,
                             motors_on?"ON ":"WAIT"); }
                else snprintf(buf,sizeof(buf),"D:---- MTR:%s",
                              motors_on?"ON ":"WAIT");
                oled_text(0, 4, buf);
            } else {
                snprintf(buf,sizeof(buf),"  Motor: STOP      ");
                oled_text(0, 3, buf);
            }
        }

        /* 1.54" LCD 数据刷新 (运行时, 每 500ms) */
        if (running && dc == 0) {
            static int   last_raw = -1;
            float err = tracker_get_error();
            char buf[20], bits[9];

            /* IR 传感器 (仅变化时更新) */
            if (raw != (uint8_t)last_raw) {
                last_raw = raw;
                for(int i=0;i<8;i++) bits[i]=(raw&(1<<i))?'1':'0'; bits[8]=0;
                LCD_SetAsciiFont(&ASCII_Font24); LCD_SetColor(LCD_YELLOW); LCD_SetBackColor(LCD_BLACK);
                LCD_DisplayString(46, 50, "        ");
                LCD_DisplayString(46, 50, bits);
            }

            /* 偏离量 */
            float dev_mm = err * 15.0f;
            snprintf(buf,sizeof(buf),"%+5.1f mm",dev_mm);
            LCD_SetAsciiFont(&ASCII_Font20); LCD_SetColor(LCD_WHITE); LCD_SetBackColor(LCD_BLACK);
            LCD_DisplayString(54, 82, "            ");
            LCD_DisplayString(54, 82, buf);

            /* 方向 (先清再写, 避免重影) */
            LCD_SetColor(LCD_BLACK); LCD_FillRect(30, 112, 180, 30);
            LCD_SetAsciiFont(&ASCII_Font24);
            if (err < -0.5f)      { LCD_SetColor(LCD_RED);   LCD_DisplayString(40, 115, "<< LEFT"); }
            else if (err > 0.5f)  { LCD_SetColor(LCD_GREEN); LCD_DisplayString(140, 115, "RIGHT >>"); }
            else                  { LCD_SetColor(LCD_CYAN);  LCD_DisplayString(60, 115, "STRAIGHT"); }

            /* 偏离指示条 (只画红点, 灰底已画) */
            static int last_bar = -1;
            int bar_x = (int)(120.0f + err * 40.0f);
            if(bar_x<20)bar_x=20; if(bar_x>220)bar_x=220;
            if (bar_x != last_bar) {
                /* 清除旧点 */
                if (last_bar > 0) { LCD_SetColor(LCD_GREY); LCD_FillRect(last_bar-4, 147, 8, 14); }
                /* 画新点 */
                LCD_SetColor(LCD_RED); LCD_FillRect(bar_x-4, 147, 8, 14);
                last_bar = bar_x;
            }

            /* 底部信息 */
            int cnt=0; for(int i=0;i<8;i++) cnt+=(raw>>i)&1;
            LCD_SetAsciiFont(&ASCII_Font16); LCD_SetColor(LCD_WHITE); LCD_SetBackColor(LCD_BLACK);
            snprintf(buf,sizeof(buf),"LINE:%d/8 SPD:%d  ",cnt,base_speed);
            LCD_DisplayString(6, 220, buf);

        }
    }
}

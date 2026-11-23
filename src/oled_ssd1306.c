/**
 * SSD1306 OLED 驱动实现
 * =====================
 *
 * 通信协议 (I2C):
 *   每个 I2C 传输的第 1 字节是"控制字节"(Control Byte):
 *     0x00 = 接下来的字节是命令 (Command)
 *     0x40 = 接下来的字节是显示数据 (GDDRAM Data)
 *
 *   发送命令: [I2C地址] [0x00] [命令字节]
 *   发送数据: [I2C地址] [0x40] [数据0] [数据1] ...
 *
 *   STM32 HAL 约定:
 *     HAL_I2C_Master_Transmit() 的地址参数是 7-bit 地址
 *     左移 1 位的结果，所以 0x3C << 1 = 0x78
 */

#include "oled_ssd1306.h"
#include "delay.h"
#include "stm32f4xx_hal.h"

#define OLED_ADDR  ((uint8_t)(0x3C << 1))  /* I2C 7bit->8bit 地址 */

static I2C_HandleTypeDef oled_i2c;  /* HAL I2C 句柄（模块内部使用） */

/* ================================================================
 * 底层 I2C 通信函数
 * ================================================================ */

/** 发送一个命令字节给 SSD1306 */
static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};  /* [控制字节=命令] [命令码] */
    HAL_I2C_Master_Transmit(&oled_i2c, OLED_ADDR, buf, 2, 100);
}

/** 设置 GDDRAM 写入位置（即"光标"） */
static void oled_set_pos(uint8_t page, uint8_t col)
{
    oled_cmd(0xB0 | (page & 7));       /* 设置页地址 (0xB0-0xB7) */
    oled_cmd(0x00 | (col & 0x0F));      /* 设置列地址低 4 位 */
    oled_cmd(0x10 | ((col >> 4) & 0x0F)); /* 设置列地址高 4 位 */
}

/**
 * 发送显示数据（带 0x40 前缀）
 *
 * SSD1306 需要先收到 0x40 控制字节，才知道后续字节
 * 是 GDDRAM 数据。每次 I2C 传输最多 129 字节
 * (1 字节控制 + 128 字节数据)。超过 128 字节时分批发。
 */
static void oled_data(const uint8_t *data, int len)
{
    uint8_t buf[129];  /* 控制字节 + 最多 128 数据字节 */
    int sent = 0;

    while (sent < len) {
        int chunk = (len - sent > 128) ? 128 : (len - sent);
        buf[0] = 0x40;  /* 控制字节 = 数据模式 */
        for (int i = 0; i < chunk; i++)
            buf[i + 1] = data[sent + i];
        HAL_I2C_Master_Transmit(&oled_i2c, OLED_ADDR, buf, chunk + 1, 1000);
        sent += chunk;
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

void oled_init(void)
{
    /* --- 第 1 步: 初始化 I2C1 外设 --- */

    /* 开启 GPIOB 和 I2C1 的时钟（STM32 外设需要先开时钟才能用） */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* 配置 PB8 和 PB9 为 I2C1 功能 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;  /* PB8 + PB9 */
    gpio.Mode      = GPIO_MODE_AF_OD;           /* 复用开漏输出 (I2C 要求) */
    gpio.Pull      = GPIO_PULLUP;               /* 上拉 (I2C 总线需要) */
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;             /* 选择 I2C1 复用功能 */
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 配置 I2C1 参数 */
    oled_i2c.Instance             = I2C1;
    oled_i2c.Init.ClockSpeed      = 400000;     /* 400kHz 快速模式 */
    oled_i2c.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    oled_i2c.Init.OwnAddress1     = 0;
    oled_i2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    oled_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    oled_i2c.Init.OwnAddress2     = 0;
    oled_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    oled_i2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&oled_i2c);

    delay_ms(50);  /* 等待屏幕电路稳定 */

    /* --- 第 2 步: SSD1306 初始化命令序列 ---
     *
     * 这些命令来自 SSD1306 数据手册的推荐初始化流程。
     * 不是随意编的——每个命令都有具体含义。
     *
     * 关键命令说明:
     *   0xAE = 关闭显示（初始化时先关，配完再开）
     *   0x20,0x00 = 内存寻址模式 → 水平模式
     *   0xA1 = 段重映射（水平翻转，否则字是反的）
     *   0xC8 = COM 扫描方向（垂直翻转）
     *   0xA4 = 正常显示模式
     *           ⚠️ 注意是 0xA4 不是 0xA5！
     *           0xA5 = 全屏点亮（测试用），会导致蓝屏
     *   0xA6 = 正常/非反转显示 (0xA7 = 反转)
     *   0x8D,0x14 = 启用电荷泵（3.3V 必须开启）
     *   0xAF = 打开显示
     */

    uint8_t cmds[] = {
        0xAE,        /* Display OFF                          */
        0x20, 0x00,  /* Memory Addressing: Horizontal        */
        0x40,        /* Start Line = 0                       */
        0xA1,        /* Segment Remap (水平翻转)              */
        0xC8,        /* COM Scan Direction (垂直翻转)         */
        0xDA, 0x12,  /* COM Pins configuration               */
        0x81, 0x7F,  /* Contrast = 127                       */
        0xA4,        /* Display All-On Resume → 正常         */
        0xA6,        /* Normal (非反转) Display              */
        0xD9, 0xF1,  /* Pre-charge Period                    */
        0xDB, 0x40,  /* VCOMH Deselect Level                 */
        0x8D, 0x14,  /* Charge Pump Enable (3.3V 必须)       */
        0x2E,        /* Deactivate Scroll                    */
        0xAF,        /* Display ON                           */
    };

    for (uint32_t i = 0; i < sizeof(cmds); i++)
        oled_cmd(cmds[i]);

    delay_ms(10);
}

void oled_clear(void)
{
    uint8_t zero[128] = {0};  /* 128 字节全 0 = 一个全黑页 */
    for (int pg = 0; pg < 8; pg++) {
        oled_set_pos(pg, 0);
        oled_data(zero, 128);
    }
}

void oled_chess(void)
{
    uint8_t row[128];
    for (int pg = 0; pg < 8; pg++) {
        /* 生成棋盘格图案：相邻"格子"取反 */
        for (int i = 0; i < 128; i++)
            row[i] = ((i / 8 + pg) % 2) ? 0xFF : 0x00;
        oled_set_pos(pg, 0);
        oled_data(row, 128);
    }
}

/* ================================================================
 * 6x8 ASCII 字体
 *
 * 每个字符占 6 列 x 8 行像素。
 * 数据排列: 字节数组[0..5]，每个字节 = 1 列 8 像素，
 * MSB 在上（bit7 = 第 1 行）, LSB 在下（bit0 = 第 8 行）。
 *
 * 示例: '!' = {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}
 *        列0  列1  列2  列3  列4  列5
 *        ' '  ' '  '|'  ' '  ' '  ' '
 *
 * 字符范围: ASCII 32 (' ') 到 94 ('^')
 * ================================================================ */
static const uint8_t font[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /*  32 ' ' (空格) */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /*  33 '!' */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /*  34 '"' */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /*  35 '#' */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /*  36 '$' */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /*  37 '%' */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /*  38 '&' */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /*  39 ''' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /*  40 '(' */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /*  41 ')' */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /*  42 '*' */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /*  43 '+' */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /*  44 ',' */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /*  45 '-' */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /*  46 '.' */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /*  47 '/' */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /*  48 '0' */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /*  49 '1' */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /*  50 '2' */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /*  51 '3' */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /*  52 '4' */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /*  53 '5' */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /*  54 '6' */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /*  55 '7' */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /*  56 '8' */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /*  57 '9' */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /*  58 ':' */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /*  59 ';' */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /*  60 '<' */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /*  61 '=' */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /*  62 '>' */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /*  63 '?' */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /*  64 '@' */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /*  65 'A' */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /*  66 'B' */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /*  67 'C' */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /*  68 'D' */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /*  69 'E' */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /*  70 'F' */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /*  71 'G' */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /*  72 'H' */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /*  73 'I' */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /*  74 'J' */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /*  75 'K' */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /*  76 'L' */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /*  77 'M' */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /*  78 'N' */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /*  79 'O' */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /*  80 'P' */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /*  81 'Q' */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /*  82 'R' */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /*  83 'S' */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /*  84 'T' */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /*  85 'U' */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /*  86 'V' */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /*  87 'W' */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /*  88 'X' */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /*  89 'Y' */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /*  90 'Z' */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /*  91 '[' */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /*  92 '\' */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /*  93 ']' */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /*  94 '^' */
};

/** 在 (col, page) 显示一个 6x8 点阵字符 */
static void oled_char(uint8_t col, uint8_t page, char c)
{
    if (c < 32 || c > 94) return;  /* 超出字体范围，不显示 */

    const uint8_t *glyph = font[c - 32];  /* 查表获取字形数据 */

    oled_set_pos(page, col);

    /* 构建 I2C 数据: [控制字节 0x40] + [6 字节字形] */
    uint8_t buf[7];
    buf[0] = 0x40;
    for (int i = 0; i < 6; i++)
        buf[i + 1] = glyph[i];

    HAL_I2C_Master_Transmit(&oled_i2c, OLED_ADDR, buf, 7, 100);
}

void oled_text(uint8_t col, uint8_t page, const char *str)
{
    while (*str) {
        oled_char(col, page, *str);
        col += 6;  /* 每个字符宽 6 像素，写完后光标右移 6 列 */
        str++;
    }
}

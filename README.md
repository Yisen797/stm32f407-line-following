# STM32F407 三轮寻迹小车

基于 **STM32F407ZGT6**（HSE 168 MHz）与 PlatformIO / STM32Cube HAL 的三轮循迹小车。

## 功能

- 多路红外（IR）循迹传感器，加权偏差计算 + PID 循迹
- 编码器测速、差速电机控制
- 1.54 寸 SPI LCD：系统状态、IR 数据、偏离量、方向
- 0.96 寸 SSD1306 OLED：状态 + 超声波距离
- 按键 PA15：启动 / 停止（电机延迟 2 s 启动）
- 蜂鸣器提示

## 目录结构

```
src/
  main.c          主循环：按键、显示、循迹、电机输出
  sensor.c/h      红外循迹传感器
  tracker.c/h     循迹算法（偏差 -> 转向）
  pid.c/h         PID 控制器
  motor.c/h       电机 PWM 驱动
  encoder.c/h     编码器测速
  ultrasonic.c/h  超声波测距
  lcd_spi_154.*   1.54 寸 SPI LCD + 字库/图片
  oled_ssd1306.*  0.96 寸 OLED
  button.c/h, buzzer.c/h, delay.c/h
platformio.ini
```

## 编译与烧录

```bash
pio run              # 编译
pio run -t upload    # 使用 ST-Link 烧录
pio device monitor   # 串口监视，115200
```

## 调参提示

- `PULSES_PER_REV`（电机一圈脉冲数）需按实际电机与减速比实测校准。
- PID 参数（KP/KD）与赛道材质、车速强相关，换场地后需重新整定。

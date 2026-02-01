[English](README_en.md) | [简体中文](README.md)

# BH45B1225 驱动

BH45B1225 是一款高性价比的 24 位 Delta-Sigma ADC，集成可编程增益放大器（PGA，1\~128 倍）、12 位 DAC、温度传感器和多路输入通道（4 路单端 / 2 路差分）。支持内部 1.25V 基准电压，通过 I2C 接口配置，数据速率可调（5\~1600Hz），适用于精密测量、传感器采集、工业控制等场景。


## 移植

实现 3 个函数：

```c
// 写 - data 格式: [寄存器地址][数据...]
int bh45b1225_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len) {
    return HAL_I2C_Master_Transmit(&hi2c1, dev_addr, data, len, 1000);
}

// 读
int bh45b1225_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len) {
    if (HAL_I2C_Master_Transmit(&hi2c1, dev_addr, &reg, 1, 1000) != HAL_OK) return -1;
    return HAL_I2C_Master_Receive(&hi2c1, dev_addr, data, len, 1000);
}

// 延时
void bh45b1225_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

## 使用

```c
bh45b1225_dev_t sensor;

// 初始化（8位地址，如 0xD0）
bh45b1225_init(&sensor, 0xD0, bh45b1225_i2c_write, bh45b1225_i2c_read, bh45b1225_delay_ms);

// 配置差分输入 (AN0-AN1)
bh45b1225_set_input_channel(&sensor, BH45B1225_IN1_AN0, BH45B1225_IN2_AN1);
bh45b1225_reset_adc_filter(&sensor);
bh45b1225_start_conversion(&sensor);

// 读取
while (1) {
    bool complete;
    if (bh45b1225_check_eoc(&sensor, &complete) == 0 && complete) {
        int32_t raw;
        bh45b1225_read_data(&sensor, &raw);
        bh45b1225_clear_eoc(&sensor);

        float voltage = bh45b1225_code_to_voltage(raw, 1.241f, 1.0f);
    }
}
```

## API

| 函数 | 功能 |
|------|------|
| `bh45b1225_init()` | 初始化 |
| `bh45b1225_set_input_channel()` | 输入通道 |
| `bh45b1225_set_pga_gain()` | 增益 (1~128) |
| `bh45b1225_set_data_rate()` | 速率 (5~1600Hz) |
| `bh45b1225_set_dac_output()` | DAC 输出 |
| `bh45b1225_reset_adc_filter()` | 复位滤波器 |
| `bh45b1225_start_conversion()` | 启动 ADC |
| `bh45b1225_read_data()` | 读数据 |
| `bh45b1225_clear_eoc()` | 清 EOC 标志 |
| `bh45b1225_code_to_voltage()` | 转电压输出 |

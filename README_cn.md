[English](README.md)

# BH45B1225 驱动

BH45B1225 是一款高性价比的 24 位 Delta-Sigma ADC，集成可编程增益放大器（PGA，1\~128 倍）、12 位 DAC、温度传感器和多路输入通道（4 路单端 / 2 路差分）。支持内部 1.25V 基准电压，通过 I2C 接口配置，数据速率可调（5\~1600Hz），适用于精密测量、传感器采集、工业控制等场景。

![数据手册封面](./img/cn.png)

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

### 基础函数
| 函数 | 功能 |
|------|------|
| `bh45b1225_init()` | 初始化设备 |
| `bh45b1225_set_vcm()` | 使能/除能 VCM |
| `bh45b1225_set_vref_source()` | 设置 ADC 参考电压源 (内部/外部) |

### 输入与增益配置
| 函数 | 功能 |
|------|------|
| `bh45b1225_set_input_channel()` | 配置输入通道 (IN1/IN2) |
| `bh45b1225_set_inx_polarity()` | 通过 INX 位交换输入极性 |
| `bh45b1225_set_pga_gain()` | 设置 PGA 总增益 (1~128) |

### 时钟与振荡器
| 函数 | 功能 |
|------|------|
| `bh45b1225_enable_hirc()` | 使能 HIRC 内部振荡器 |
| `bh45b1225_check_hirc_stable()` | 检查 HIRC 振荡器是否稳定 |

### ADC 配置
| 函数 | 功能 |
|------|------|
| `bh45b1225_set_data_rate()` | 设置 ADC 输出数据速率 (5~1600Hz) |
| `bh45b1225_set_adc_mode()` | 设置 ADC 工作模式 (正常/休眠/掉电) |
| `bh45b1225_set_vref_buffer()` | 使能/除能参考电压缓存 |

### ADC 操作
| 函数 | 功能 |
|------|------|
| `bh45b1225_reset_adc_filter()` | 复位 ADC 滤波器 |
| `bh45b1225_set_data_latch()` | 使能/除能数据锁存 |
| `bh45b1225_start_conversion()` | 启动 ADC 转换 |
| `bh45b1225_check_eoc()` | 检查转换是否完成 |
| `bh45b1225_read_data()` | 读取 24 位 ADC 结果 |
| `bh45b1225_clear_eoc()` | 清除 EOC 标志 |

### DAC 操作
| 函数 | 功能 |
|------|------|
| `bh45b1225_set_dac_enable()` | 使能/除能 DAC |
| `bh45b1225_set_dac_vref()` | 设置 DAC 参考电压源 (AVDD/VCM) |
| `bh45b1225_set_dac_output()` | 设置 DAC 输出值 (12 位, 0-4095) |

### 工具函数
| 函数 | 功能 |
|------|------|
| `bh45b1225_code_to_voltage()` | 将 ADC 码转换为电压 |

### 高级配置（谨慎使用）
| 函数 | 功能 |
|------|------|
| `bh45b1225_set_pwrc_opt()` | 设置 PWRC 优化位 |
| `bh45b1225_set_adcte()` | 设置 ADC 测试配置寄存器 |
| `bh45b1225_set_filter_mode()` | 设置 ADC 滤波器模式 (FLMS) |
| `bh45b1225_set_osr()` | 设置 ADC 过采样率 (OSR) |
| `bh45b1225_set_clock_div()` | 设置 ADC 时钟分频 |

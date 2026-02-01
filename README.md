[简体中文](README_cn.md)

# BH45B1225 Driver

BH45B1225 is a high-performance 24-bit Delta-Sigma ADC that integrates a programmable gain amplifier (PGA, 1\~128x), 12-bit DAC, temperature sensor, and multiple input channels (4 single-ended / 2 differential). It supports an internal 1.25V reference voltage, configurable via I2C interface, with adjustable data rate (5\~1600Hz), suitable for precision measurement, sensor acquisition, industrial control and other applications.

![数据手册封面](./img/en.png)

## Porting

Implement 3 functions:

```c
// Write - data format: [register address][data...]
int bh45b1225_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len) {
    return HAL_I2C_Master_Transmit(&hi2c1, dev_addr, data, len, 1000);
}

// Read
int bh45b1225_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len) {
    if (HAL_I2C_Master_Transmit(&hi2c1, dev_addr, &reg, 1, 1000) != HAL_OK) return -1;
    return HAL_I2C_Master_Receive(&hi2c1, dev_addr, data, len, 1000);
}

// Delay
void bh45b1225_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

## Usage

```c
bh45b1225_dev_t sensor;

// Initialize (8-bit address, e.g. 0xD0)
bh45b1225_init(&sensor, 0xD0, bh45b1225_i2c_write, bh45b1225_i2c_read, bh45b1225_delay_ms);

// Configure differential input (AN0-AN1)
bh45b1225_set_input_channel(&sensor, BH45B1225_IN1_AN0, BH45B1225_IN2_AN1);
bh45b1225_reset_adc_filter(&sensor);
bh45b1225_start_conversion(&sensor);

// Read
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

### Basic Functions
| Function | Description |
|----------|-------------|
| `bh45b1225_init()` | Initialize device |
| `bh45b1225_set_vcm()` | Enable/disable VCM |
| `bh45b1225_set_vref_source()` | Set ADC reference voltage source (internal/external) |

### Input & Gain Configuration
| Function | Description |
|----------|-------------|
| `bh45b1225_set_input_channel()` | Configure input channels (IN1/IN2) |
| `bh45b1225_set_inx_polarity()` | Set input polarity via INX bits |
| `bh45b1225_set_pga_gain()` | Set PGA total gain (1~128) |

### Clock & Oscillator
| Function | Description |
|----------|-------------|
| `bh45b1225_enable_hirc()` | Enable HIRC internal oscillator |
| `bh45b1225_check_hirc_stable()` | Check if HIRC oscillator is stable |

### ADC Configuration
| Function | Description |
|----------|-------------|
| `bh45b1225_set_data_rate()` | Set ADC output data rate (5~1600Hz) |
| `bh45b1225_set_adc_mode()` | Set ADC work mode (normal/sleep/poweroff) |
| `bh45b1225_set_vref_buffer()` | Enable/disable reference voltage buffers |

### ADC Operations
| Function | Description |
|----------|-------------|
| `bh45b1225_reset_adc_filter()` | Reset ADC filter |
| `bh45b1225_set_data_latch()` | Enable/disable data latch |
| `bh45b1225_start_conversion()` | Start ADC conversion |
| `bh45b1225_check_eoc()` | Check if conversion is complete |
| `bh45b1225_read_data()` | Read 24-bit ADC result |
| `bh45b1225_clear_eoc()` | Clear EOC flag |

### DAC Operations
| Function | Description |
|----------|-------------|
| `bh45b1225_set_dac_enable()` | Enable/disable DAC |
| `bh45b1225_set_dac_vref()` | Set DAC reference voltage source (AVDD/VCM) |
| `bh45b1225_set_dac_output()` | Set DAC output value (12-bit, 0-4095) |

### Utility Functions
| Function | Description |
|----------|-------------|
| `bh45b1225_code_to_voltage()` | Convert ADC code to voltage |

### Advanced Configuration (Use with caution)
| Function | Description |
|----------|-------------|
| `bh45b1225_set_pwrc_opt()` | Set PWRC optimization bits |
| `bh45b1225_set_adcte()` | Set ADC test configuration register |
| `bh45b1225_set_filter_mode()` | Set ADC filter mode (FLMS) |
| `bh45b1225_set_osr()` | Set ADC oversampling rate (OSR) |
| `bh45b1225_set_clock_div()` | Set ADC clock division |
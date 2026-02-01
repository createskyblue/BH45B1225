# BH45B1225 驱动使用说明

24位 Delta-Sigma ADC 驱动，基于 STM32 HAL 库实现。

## 快速上手

### 1. I2C 读写函数包装

使用 HAL 库时需要包装 I2C 读写函数：

```c
// I2C 写函数 - data 格式为 [寄存器地址][数据...]
int bh45b1225_i2c_write(uint8_t dev_addr, const uint8_t *data, uint16_t len) {
    return HAL_I2C_Master_Transmit(&hi2c1, dev_addr, data, len, 1000);
}

// I2C 读函数
int bh45b1225_i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len) {
    if (HAL_I2C_Master_Transmit(&hi2c1, dev_addr, &reg, 1, 1000) != HAL_OK) {
        return -1;
    }
    return HAL_I2C_Master_Receive(&hi2c1, dev_addr, data, len, 1000);
}

// 延时函数
void bh45b1225_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

### 2. 扫描 I2C 设备地址

**上电后建议先扫描 I2C 总线，确认设备地址：**

```c
// 扫描所有 7-bit 地址 (0x00-0x7F)
for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 16; col++) {
        uint8_t addr_7bit = (row << 4) | col;
        uint8_t addr_8bit = addr_7bit << 1;

        // 跳过保留地址
        if (addr_7bit < 0x08 || (addr_7bit > 0x77 && addr_7bit < 0x78)) {
            continue;
        }

        uint8_t test_val;
        if (HAL_I2C_Mem_Read(&hi2c1, addr_8bit, 0x00, I2C_MEMADD_SIZE_8BIT, &test_val, 1, 50) == HAL_OK) {
            printf("Found device at 0x%02X (7-bit: 0x%02X)\r\n", addr_8bit, addr_7bit);
        }
    }
}
```

常见地址：`0xD0` (7-bit: 0x68) 或 `0xA0` (7-bit: 0x50)，取决于 A1/A2/XTSB 引脚配置。

### 3. 初始化传感器

```c
bh45b1225_dev_t sensor;
uint8_t working_addr = 0xD0;  // 使用扫描到的地址

if (bh45b1225_init(&sensor, working_addr, bh45b1225_i2c_write, bh45b1225_i2c_read, bh45b1225_delay_ms) != 0) {
    printf("Sensor initialization failed\r\n");
    while(1);
}
```

初始化完成后，以下配置已自动设置：
- HIRC 内部时钟已使能并稳定
- VCM（共模电压，约 1.25V）已使能
- 数据速率：400 Hz
- 增益：1 倍
- ADCTE 寄存器：0xE7（**不要修改**）

### 4. 配置 DAC 作为偏置电压（可选）

**芯片的共模抑制性能比较拉跨，建议用 DAC 提供稳定的偏置电压：**

```c
// 设置 DAC 参考电压为 VCM (1.25V)
bh45b1225_set_dac_vref(&sensor, BH45B1225_DAC_VREF_VCM);

// 设置 DAC 输出值（0-4095 对应 0-1.25V）
// 例如 1650 ≈ 0.5V 左右
bh45b1225_set_dac_output(&sensor, 1650);

// 使能 DAC
bh45b1225_set_dac_enable(&sensor, true);
```

### 5. 配置输入通道

```c
// 方式1：单端输入（示例负端接 DAC 输出）
bh45b1225_set_input_channel(&sensor, BH45B1225_IN1_AN0, BH45B1225_IN2_VDACO);

// 方式2：差分输入
bh45b1225_set_input_channel(&sensor, BH45B1225_IN1_AN0, BH45B1225_IN2_AN1);

// 方式3：温度传感器
bh45b1225_set_input_channel(&sensor, BH45B1225_IN1_VTSP, BH45B1225_IN2_VTSN);
```

**推荐使用方式1**，将 DAC 输出作为负端参考，可以有效改善共模抑制问题。

### 6. 复位滤波器并启动转换

```c
// 改变设置后必须复位滤波器
bh45b1225_reset_adc_filter(&sensor);

// 启动 ADC
bh45b1225_start_conversion(&sensor);
```

### 7. 读取数据

```c
while (1) {
    bool complete = false;
    int ret = bh45b1225_check_eoc(&sensor, &complete);

    if (ret == 0 && complete) {
        int32_t raw_adc;
        ret = bh45b1225_read_data(&sensor, &raw_adc);

        // 清除 EOC 标志
        bh45b1225_clear_eoc(&sensor);

        if (ret == 0) {
            // Vref ≈ 1.241V，增益 = 1
            float voltage = bh45b1225_code_to_voltage(raw_adc, 1.241f, 1.0f);
            printf("ADC,mV: %ld, %.4f\r\n", raw_adc, voltage * 1000);
        }
    }
}
```

---

## 重点注意事项

### 1. 共模抑制性能较差 ⚠️

这个芯片的 CMRR 表现不太理想，建议：
- 使用 DAC 输出（VDACO）作为负端参考，而不是直接接 GND
- 保持信号源共模电压接近 VCM（1.25V）
- 单端测量时推荐配置：`AN0` vs `VDACO`

### 2. I2C 地址格式

驱动使用**8位地址格式**（包含读写位）：
- 7-bit 地址 0x68 → 传入 `0xD0`
- 7-bit 地址 0x50 → 传入 `0xA0`

### 3. 参考电压

- **内部参考**：VCM ≈ 1.241V（实测值，芯片典型值 1.25V）
- **外部参考**：通过 VREFP/VREFN 引脚提供
- 计算 `voltage` 时需使用实际测量的 Vref 值

### 4. 数据锁存机制

驱动已自动处理数据锁存，调用 `bh45b1225_read_data()` 时：
1. 自动开启锁存（ADCDL=1）
2. 分别读取 ADRL/ADRM/ADRH 三个寄存器
3. 自动关闭锁存（ADCDL=0）

### 5. 清除 EOC 标志

读取数据后**必须手动清除 EOC 标志**，否则下一次转换完成时标志位不会更新：

```c
bh45b1225_clear_eoc(&sensor);  // 必须调用
```

### 6. ADCTE 寄存器禁止修改

```c
// ADCTE 已在 init() 中设置为 0xE7，不要修改！
// bh45b1225_set_adcte(&sensor, 0xE7);  // 已自动完成
```

### 7. 改变配置后需复位滤波器

修改以下任一配置后，必须调用 `bh45b1225_reset_adc_filter()`：
- 输入通道
- PGA 增益
- 数据速率

```c
bh45b1225_set_pga_gain(&sensor, BH45B1225_GAIN_64);
bh45b1225_reset_adc_filter(&sensor);  // 必须！
```

---

## 常见问题排查

### I2C 扫描不到设备

1. 检查硬件连接（SDA、SCL、VDD、GND）
2. 检查上拉电阻（4.7kΩ 典型值）
3. 降低 I2C 时钟频率（尝试 50kHz）
4. 确认电源电压（2.4V-5.5V）

### 读数为 0 或异常

1. 确认是否调用了 `bh45b1225_start_conversion()`
2. 确认是否复位了滤波器
3. 检查输入通道配置是否正确
4. 确认是否清除了 EOC 标志

### 数据不稳定

1. 降低数据速率（如改为 10Hz 或 50Hz）
2. 使用 DAC 作为偏置参考（改善共模抑制）
3. 检查电源去耦电容（AVDD 0.1μF，VCM 1μF）

---

## API 快速索引

| 函数 | 说明 |
|------|------|
| `bh45b1225_init()` | 初始化传感器 |
| `bh45b1225_set_input_channel()` | 配置输入通道 |
| `bh45b1225_set_pga_gain()` | 设置增益（1/2/4/8/16/32/64/128） |
| `bh45b1225_set_dac_output()` | 设置 DAC 输出（0-4095） |
| `bh45b1225_set_dac_enable()` | 使能/除能 DAC |
| `bh45b1225_reset_adc_filter()` | 复位滤波器 |
| `bh45b1225_start_conversion()` | 启动 ADC |
| `bh45b1225_check_eoc()` | 检查转换完成 |
| `bh45b1225_read_data()` | 读取 ADC 数据 |
| `bh45b1225_clear_eoc()` | 清除 EOC 标志 |
| `bh45b1225_code_to_voltage()` | ADC 码转电压 |

---

## 文件说明

| 文件 | 说明 |
|------|------|
| [bh45b1225.h](bh45b1225.h) | 驱动头文件 |
| [bh45b1225.c](bh45b1225.c) | 驱动实现 |
| [[ADC]中文BH45B1225v120.pdf]([ADC]中文BH45B1225v120.pdf) | 官方数据手册 |

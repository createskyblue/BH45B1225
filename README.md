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

## Understanding Common-Mode Voltage and Input Configuration

### What is Common-Mode Voltage

The most important thing to understand about this ADC is that it **only measures the voltage difference** between its two input pins, INP (positive input) and INN (negative input). The conversion result is Vdiff = VINP - VINN, nothing more and nothing less. However, while the ADC outputs this difference, the internal circuitry must still process the absolute voltage level of both pins relative to the ADC ground. This absolute level is what we call the common-mode voltage (don't confuse this with the chip's VCM pin), calculated as V_cm = (VINP + VINN) / 2.

Think of it this way: if you have two people standing on different steps of a staircase, the differential voltage is the height difference between them, while the common-mode voltage is the average height of the step they're standing on from the floor. The ADC cares about both - it needs to know the difference (that's your measurement), but it also needs both inputs to be at absolute voltage levels that its internal circuitry can handle.

### Common-Mode Voltage Limits of BH45B1225

The BH45B1225 has strict limits on the common-mode voltage range, which depends on your supply voltage and reference voltage configuration. If the common-mode voltage falls outside this allowable range, the ADC cannot accurately process the signals, resulting in incorrect readings or potential chip damage. This is why the chip provides a dedicated VCM pin (this is a physical pin name), which outputs AVDD/2 (for example, when AVDD=2.5V, the VCM pin outputs 1.25V), serving as a safe "middle ground" reference for your measurements. We strongly recommend using this VCM pin instead of ground for your reference connection.

### Single-Ended Measurement Configuration

For single-ended measurements, you need to configure INP to AN1 (your signal) and INN to the chip's VCM pin, then connect your signal source to AN1 and your signal ground to the VCM pin (not to ADC ground!). This configuration works because the ADC measures VINP - VINN = AN1_voltage - VCM_pin_voltage, which gives you your actual signal voltage. Crucially, this keeps both inputs within the acceptable common-mode range since the VCM pin sits at the middle of the ADC's valid input range.

**We strongly recommend verifying your setup with a multimeter before trusting the ADC readings:** Measure the voltage between AN1 and VCM (this should match your signal), then measure AN1-to-GND and VCM-to-GND separately. Calculate the common-mode voltage manually as (V_AN1_GND + V_VCM_GND) / 2 and verify it's within the chip's specified range. If you instead connect INN directly to ground, the common-mode voltage becomes approximately half of your signal voltage, which may be too low for the ADC to handle properly, especially with small signals near ground. Many beginners make this mistake and wonder why their readings are inaccurate or unstable - the ADC simply cannot operate correctly when the common-mode voltage is out of range.

### Differential Measurement Configuration

For differential measurements, configure INP to AN1 and INN to AN2, then connect your two signal sources to these pins. The ADC will output the difference between them: Vdiff = VAN1 - VAN2. However, even in differential mode, both signal sources must have a common-mode voltage that falls within the ADC's allowable range. This is where many users encounter problems: if you're using a bridge sensor powered by 5V where both outputs sit around 2.5V, this common-mode level might be acceptable, but if your sensor is powered from a different voltage or has an offset, you may need to shift the signals into the valid range.

**Always verify with a multimeter:** Measure AN1-to-GND, AN2-to-GND, and AN1-to-AN2 voltages, then calculate the actual common-mode voltage as (V_AN1_GND + V_AN2_GND) / 2. If this value is outside the specified range, you need to connect one of your signal sources to VCM through an appropriate resistor (typically in the range of 1kΩ to 100kΩ, depending on your signal source impedance) to shift the common-mode level. Alternatively, redesign your signal conditioning circuit to reference both signals to VCM. Many differential measurement failures stem from ignoring the common-mode voltage requirement - the ADC will output garbage data even though the differential signal itself looks perfect on an oscilloscope, because the internal circuitry cannot handle the absolute voltage levels present at the inputs.

### Practical Example: Differential Measurement with Different Ground Connections

Let's walk through a concrete example to illustrate why proper common-mode voltage matters. Suppose you have a signal source with differential outputs (positive and negative) and you set it to output 1V. Configure INP to AN1 and INN to AN2, then connect AN1 to the signal positive and AN2 to the signal negative. Now let's observe what happens with three different grounding configurations.

**Configuration 1: Floating ground (no connection between signal ground and chip ground)**
In this setup, the signal ground is not connected to anything - it's floating relative to the ADC chip. You'll observe that the ADC readings fluctuate wildly and are completely unstable. This happens because the common-mode voltage is floating and undefined. External electromagnetic interference couples into the floating wires, causing the absolute voltage levels of both AN1 and AN2 to drift unpredictably. Even though the differential signal (the voltage difference between them) is still 1V, the ADC cannot lock onto this difference because its internal circuitry relies on stable common-mode voltage to function. The floating common-mode voltage wanders outside the ADC's valid operating range, so the output is essentially random noise.

**Configuration 2: Signal ground connected to chip ground (GND)**

Now connect your signal ground to the ADC chip's GND pin. You'll notice that the interference instantly disappears and the readings become stable - the ADC correctly measures close to 1V. But let's look at the actual numbers:

When the signal source is set to +1V, signal+ to chip ground is 1.0V, signal- to chip ground is 0V. The common-mode voltage = (1.0V + 0V) / 2 = 0.5V. While this works for positive signals, if you try to measure negative voltages (for example, set the signal source to output -1V), signal+ to chip ground needs to be -1V, signal- to chip ground is 0V. The common-mode voltage = (-1V + 0V) / 2 = -0.5V, and signal+ = -1V is already outside the ADC's input range (ADC cannot measure negative voltages).

**Configuration 3: Signal ground connected to chip VCM pin (NOT chip GND)**

Finally, disconnect the signal ground from chip GND and connect it to the chip's VCM pin instead (in our test setup, the VCM pin outputs 1.25V). You'll observe that for positive signals (like the original 1V example), the ADC still measures accurately, just like in Configuration 2. But now something magical happens: when you apply negative differential voltages, the ADC correctly measures them!

Let's use the actual measured values from your tests to illustrate. In the test, the VCM pin outputs 1.25V (relative to chip ground, this value doesn't change), and the external signal source's ground is connected to the VCM pin.

**When the signal source is set to +1V:**

- Signal- (connected to VCM) to chip ground: 1.25V
- Signal+ to chip ground: 2.25V
- Differential voltage: 2.25V - 1.25V = 1V
- Common-mode voltage: (2.25V + 1.25V) / 2 = 1.75V
- Both pin voltages are within the ADC's allowable range ✓

**When the signal source is set to -1V:**

- Signal- (connected to VCM) to chip ground: 1.25V
- Signal+ to chip ground: 0.25V
- Differential voltage: 0.25V - 1.25V = -1V
- Common-mode voltage: (0.25V + 1.25V) / 2 = 0.75V
- Both pin voltages are still within the ADC's allowable range ✓

Compare this with Configuration 2 (signal ground connected to GND, assuming GND=0V):

**When the signal source is set to +1V:**

- Signal+ to chip ground: 1.0V
- Signal- to chip ground: 0V
- Differential voltage: 1V - 0V = 1V
- Common-mode voltage: (1.0V + 0V) / 2 = 0.5V
- Works fine, measurement is accurate ✓

**When the signal source is set to -1V:**

- Signal+ to chip ground: -1V (already outside ADC input range ✗)
- Signal- to chip ground: 0V
- Differential voltage: -1V - 0V = -1V
- Common-mode voltage: (-1V + 0V) / 2 = -0.5V
- Signal+ is already negative, outside the ADC's input range, cannot measure ✗

**Why does this matter?**

Differential ADCs have input range limits on each individual pin, not just for the differential voltage between them.

- With ground reference in Configuration 2, you can measure positive differential voltages, but cannot measure negative differential voltages (because that would require one pin to go negative)
- With VCM pin reference in Configuration 3, the signal swings around VCM (1.25V). Whether it's +1V or -1V, the absolute voltages of signal+ and signal- are both between 0V and 2.5V, staying within the ADC's allowable range, achieving true bipolar measurement capability

This is why applications like bridge sensors and audio signals must use the VCM pin as reference.



/**
 * @file bh45b1225.c
 * @brief BH45B1225 24位Delta-Sigma ADC驱动程序实现
 * @note 基于BH45B1225数据手册 Rev 1.21重新实现
 */

#include "bh45b1225.h"
#include <string.h>

/* =========================== 私有方法实现 =========================== */

static int write_reg(bh45b1225_dev_t *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL || dev->i2c_write == NULL) {
        return -1;
    }
    return dev->i2c_write(dev->dev_addr, reg, &value, 1);
}

static int read_reg(bh45b1225_dev_t *dev, uint8_t reg, uint8_t *value)
{
    if (dev == NULL || dev->i2c_read == NULL || value == NULL) {
        return -1;
    }
    return dev->i2c_read(dev->dev_addr, reg, value, 1);
}

/* =========================== 公共API实现 =========================== */

int bh45b1225_init(bh45b1225_dev_t *dev, uint8_t dev_addr,
                   bh45b1225_i2c_write_t write_func,
                   bh45b1225_i2c_read_t read_func)
{
    if (dev == NULL || write_func == NULL || read_func == NULL) {
        return -1;
    }

    memset(dev, 0, sizeof(bh45b1225_dev_t));

    dev->dev_addr = dev_addr;
    dev->i2c_write = write_func;
    dev->i2c_read = read_func;
    dev->write_reg = write_reg;
    dev->read_reg = read_reg;

    // 默认配置: 使能VCM, ADC正常工作
    uint8_t pwrc = BH45B1225_PWRC_VCMEN;
    int ret = dev->write_reg(dev, BH45B1225_REG_PWRC, pwrc);
    if (ret != 0) {
        return ret;
    }

    // 设置ADCTE寄存器为固定值 1110_0111B (0xE7)
    ret = dev->write_reg(dev, BH45B1225_REG_ADCTE, 0xE7);
    if (ret != 0) {
        return ret;
    }

    // 设置默认增益: PGAGN=1, ADGN=1 (总增益=1)
    uint8_t pgac0 = (0 << BH45B1225_VGS_SHIFT) |   // VREFGN=1
                    (0 << BH45B1225_AGS_SHIFT) |   // ADGN=1
                    (0 << BH45B1225_PGS_SHIFT);    // PGAGN=1
    ret = dev->write_reg(dev, BH45B1225_REG_PGAC0, pgac0);

    return ret;
}

void bh45b1225_deinit(bh45b1225_dev_t *dev)
{
    if (dev == NULL) {
        return;
    }
    memset(dev, 0, sizeof(bh45b1225_dev_t));
}

int bh45b1225_set_pga_gain(bh45b1225_dev_t *dev, bh45b1225_total_gain_t gain)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    if (gain > BH45B1225_GAIN_128) {
        return -2;  // 无效增益
    }

    // 读取当前PGAC0配置
    uint8_t pgac0;
    int ret = dev->read_reg(dev, BH45B1225_REG_PGAC0, &pgac0);
    if (ret != 0) {
        return ret;
    }

    // 根据总增益计算PGAGN和ADGN
    // 总增益 = PGAGN × ADGN
    // Gain=1: PGAGN=1, ADGN=1
    // Gain=2: PGAGN=2, ADGN=1
    // Gain=4: PGAGN=4, ADGN=1
    // ...
    // Gain=128: PGAGN=64, ADGN=2
    bh45b1225_pgagn_t pgagn;
    bh45b1225_adgn_t adgn;

    if (gain == BH45B1225_GAIN_128) {
        pgagn = BH45B1225_PGAGN_64;
        adgn = BH45B1225_ADGN_2;
    } else {
        pgagn = (bh45b1225_pgagn_t)gain;
        adgn = BH45B1225_ADGN_1;
    }

    // 更新PGAC0寄存器
    pgac0 &= ~BH45B1225_AGS_MASK;
    pgac0 &= ~BH45B1225_PGS_MASK;
    pgac0 |= (adgn << BH45B1225_AGS_SHIFT);
    pgac0 |= (pgagn << BH45B1225_PGS_SHIFT);

    return dev->write_reg(dev, BH45B1225_REG_PGAC0, pgac0);
}

int bh45b1225_set_input_channel(bh45b1225_dev_t *dev,
                                bh45b1225_in1_select_t in1_pos,
                                bh45b1225_in2_select_t in2_neg)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前PGACS配置
    uint8_t pgacs;
    int ret = dev->read_reg(dev, BH45B1225_REG_PGACS, &pgacs);
    if (ret != 0) {
        return ret;
    }

    // 更新CHSN (BIT5~BIT3) 和 CHSP (BIT2~BIT0)
    pgacs &= ~BH45B1225_CHSN_MASK;
    pgacs &= ~BH45B1225_CHSP_MASK;
    pgacs |= (in2_neg << BH45B1225_CHSN_SHIFT);
    pgacs |= (in1_pos << BH45B1225_CHSP_SHIFT);

    return dev->write_reg(dev, BH45B1225_REG_PGACS, pgacs);
}

int bh45b1225_set_inx_polarity(bh45b1225_dev_t *dev, uint8_t inx_value)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    if (inx_value > 3) {
        return -2;  // INX[1:0]范围0-3
    }

    // 读取当前PGAC1配置
    uint8_t pgac1;
    int ret = dev->read_reg(dev, BH45B1225_REG_PGAC1, &pgac1);
    if (ret != 0) {
        return ret;
    }

    // 更新INX位 (BIT5~BIT4)
    pgac1 &= ~(BH45B1225_INX1_MASK | BH45B1225_INX0_MASK);
    pgac1 |= ((inx_value << 4) & (BH45B1225_INX1_MASK | BH45B1225_INX0_MASK));

    return dev->write_reg(dev, BH45B1225_REG_PGAC1, pgac1);
}

int bh45b1225_set_vcm(bh45b1225_dev_t *dev, bool enable)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前PWRC配置
    uint8_t pwrc;
    int ret = dev->read_reg(dev, BH45B1225_REG_PWRC, &pwrc);
    if (ret != 0) {
        return ret;
    }

    // 更新VCMEN位
    if (enable) {
        pwrc |= BH45B1225_PWRC_VCMEN;
    } else {
        pwrc &= ~BH45B1225_PWRC_VCMEN;
    }

    return dev->write_reg(dev, BH45B1225_REG_PWRC, pwrc);
}

int bh45b1225_start_conversion(bh45b1225_dev_t *dev)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // ADCR0的ADRST位写1可以触发转换复位
    // 实际转换是自动进行的，这个芯片是Delta-Sigma ADC，持续转换

    // 使能ADC (如果之前被关闭)
    uint8_t adcr0;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR0, &adcr0);
    if (ret != 0) {
        return ret;
    }

    // 清除ADOFF位 (打开ADC电源)
    adcr0 &= ~BH45B1225_ADCR0_ADOFF;

    // 清除ADSL位 (退出休眠)
    adcr0 &= ~BH45B1225_ADCR0_ADSL;

    return dev->write_reg(dev, BH45B1225_REG_ADCR0, adcr0);
}

int bh45b1225_check_eoc(bh45b1225_dev_t *dev, bool *complete)
{
    if (dev == NULL || dev->read_reg == NULL || complete == NULL) {
        return -1;
    }

    uint8_t adcr1;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR1, &adcr1);
    if (ret != 0) {
        return ret;
    }

    *complete = (adcr1 & BH45B1225_ADCR1_EOC) ? true : false;
    return 0;
}

int bh45b1225_read_data(bh45b1225_dev_t *dev, int32_t *data)
{
    if (dev == NULL || dev->i2c_read == NULL || data == NULL) {
        return -1;
    }

    uint8_t buffer[3];
    int ret = dev->i2c_read(dev->dev_addr, BH45B1225_REG_ADRL, buffer, 3);
    if (ret != 0) {
        return ret;
    }

    // 组合24位数据 (大端格式)
    // ADRL = Bit7~Bit0 (低8位)
    // ADRM = Bit15~Bit8 (中8位)
    // ADRH = Bit23~Bit16 (高8位)
    *data = (int32_t)((buffer[2] << 16) | (buffer[1] << 8) | buffer[0]);

    return 0;
}

float bh45b1225_code_to_voltage(int32_t adc_code, float vref, float total_gain)
{
    // 24位有符号数: 范围 -8388608 ~ +8388607
    // Voltage = (adc_code / 8388608.0) * (vref / total_gain)
    return ((float)adc_code / 8388608.0f) * (vref / total_gain);
}

int bh45b1225_set_osr(bh45b1225_dev_t *dev, bh45b1225_osr_t osr)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    if (osr > 8) {
        return -2;  // 无效OSR值 (最大值8对应OSR_128)
    }

    // 读取当前ADCR0配置
    uint8_t adcr0;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR0, &adcr0);
    if (ret != 0) {
        return ret;
    }

    // 更新ADOR位 (BIT4~BIT1)
    adcr0 &= ~BH45B1225_ADCR0_ADOR_MASK;
    adcr0 |= ((osr << BH45B1225_ADCR0_ADOR_SHIFT) & BH45B1225_ADCR0_ADOR_MASK);

    return dev->write_reg(dev, BH45B1225_REG_ADCR0, adcr0);
}

int bh45b1225_set_clock_div(bh45b1225_dev_t *dev, uint8_t div_val)
{
    if (dev == NULL || dev->write_reg == NULL) {
        return -1;
    }

    if (div_val > 31) {
        return -2;  // 超出范围
    }

    // 设置ADCS寄存器的ADCK位 (BIT4~BIT0)
    uint8_t adcs = div_val & BH45B1225_ADCS_ADCK_MASK;

    return dev->write_reg(dev, BH45B1225_REG_ADCS, adcs);
}

int bh45b1225_set_vref_source(bh45b1225_dev_t *dev, bh45b1225_vref_t vref)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前ADCR0配置
    uint8_t adcr0;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR0, &adcr0);
    if (ret != 0) {
        return ret;
    }

    // 更新VREFS位
    if (vref == BH45B1225_VREF_EXTERNAL) {
        adcr0 |= BH45B1225_ADCR0_VREFS;
    } else {
        adcr0 &= ~BH45B1225_ADCR0_VREFS;
    }

    return dev->write_reg(dev, BH45B1225_REG_ADCR0, adcr0);
}

int bh45b1225_set_dac_output(bh45b1225_dev_t *dev, uint16_t value)
{
    if (dev == NULL || dev->write_reg == NULL) {
        return -1;
    }

    if (value > 4095) {
        return -2;  // 12位DAC，最大4095
    }

    // 分离高8位和低4位
    uint8_t dah = (value >> 4) & 0xFF;  // Bit11~Bit4
    uint8_t dal = value & 0x0F;          // Bit3~Bit0

    // 必须先写DAL再写DAH
    int ret = dev->write_reg(dev, BH45B1225_REG_DAL, dal);
    if (ret != 0) {
        return ret;
    }

    return dev->write_reg(dev, BH45B1225_REG_DAH, dah);
}

int bh45b1225_set_dac_enable(bh45b1225_dev_t *dev, bool enable)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前DACC配置
    uint8_t dacc;
    int ret = dev->read_reg(dev, BH45B1225_REG_DACC, &dacc);
    if (ret != 0) {
        return ret;
    }

    // 更新DACEN位
    if (enable) {
        dacc |= BH45B1225_DACC_DACEN;
    } else {
        dacc &= ~BH45B1225_DACC_DACEN;
    }

    return dev->write_reg(dev, BH45B1225_REG_DACC, dacc);
}

int bh45b1225_set_dac_vref(bh45b1225_dev_t *dev, bh45b1225_dac_vref_t vref)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前DACC配置
    uint8_t dacc;
    int ret = dev->read_reg(dev, BH45B1225_REG_DACC, &dacc);
    if (ret != 0) {
        return ret;
    }

    // 更新DACVRS位
    if (vref == BH45B1225_DAC_VREF_VCM) {
        dacc |= BH45B1225_DACC_DACVRS;
    } else {
        dacc &= ~BH45B1225_DACC_DACVRS;
    }

    return dev->write_reg(dev, BH45B1225_REG_DACC, dacc);
}

int bh45b1225_config_temp_sensor(bh45b1225_dev_t *dev)
{
    if (dev == NULL) {
        return -1;
    }

    // 配置CHSP=VTSP (VTSP=7), CHSN=VTSN (VTSN=7)
    return bh45b1225_set_input_channel(dev, BH45B1225_IN1_VTSP, BH45B1225_IN2_VTSN);
}

int bh45b1225_read_temperature(bh45b1225_dev_t *dev, float *temperature)
{
    if (dev == NULL || temperature == NULL) {
        return -1;
    }

    // 读取ADC值
    int32_t adc_code;
    int ret = bh45b1225_read_data(dev, &adc_code);
    if (ret != 0) {
        return ret;
    }

    // 温度传感器系数: 175μV/℃
    // 假设使用内部参考电压VCM=1.25V，增益=1
    // 温度计算公式需根据实际校准数据调整
    // 这里提供一个基本的转换逻辑
    float voltage = (float)adc_code / 8388608.0f * 1.25f;

    // 175μV/℃ = 0.000175V/℃
    // 温度 = voltage / 0.000175
    *temperature = voltage / 0.000175f;

    return 0;
}

int bh45b1225_set_adc_mode(bh45b1225_dev_t *dev, bh45b1225_adc_mode_t mode)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前ADCR0配置
    uint8_t adcr0;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR0, &adcr0);
    if (ret != 0) {
        return ret;
    }

    // 根据模式设置ADOFF和ADSL位
    switch (mode) {
        case BH45B1225_ADC_MODE_POWEROFF:
            adcr0 |= BH45B1225_ADCR0_ADOFF;  // 掉电模式
            break;
        case BH45B1225_ADC_MODE_SLEEP:
            adcr0 &= ~BH45B1225_ADCR0_ADOFF;  // 打开电源
            adcr0 |= BH45B1225_ADCR0_ADSL;    // 进入休眠
            break;
        case BH45B1225_ADC_MODE_NORMAL:
            adcr0 &= ~BH45B1225_ADCR0_ADOFF;  // 打开电源
            adcr0 &= ~BH45B1225_ADCR0_ADSL;   // 退出休眠
            break;
        default:
            return -2;  // 无效模式
    }

    return dev->write_reg(dev, BH45B1225_REG_ADCR0, adcr0);
}

int bh45b1225_reset_adc_filter(bh45b1225_dev_t *dev)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前ADCR0配置
    uint8_t adcr0;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR0, &adcr0);
    if (ret != 0) {
        return ret;
    }

    // 设置ADRST位
    adcr0 |= BH45B1225_ADCR0_ADRST;
    ret = dev->write_reg(dev, BH45B1225_REG_ADCR0, adcr0);
    if (ret != 0) {
        return ret;
    }

    // 清除ADRST位
    adcr0 &= ~BH45B1225_ADCR0_ADRST;
    return dev->write_reg(dev, BH45B1225_REG_ADCR0, adcr0);
}

int bh45b1225_set_data_latch(bh45b1225_dev_t *dev, bool enable)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前ADCR1配置
    uint8_t adcr1;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR1, &adcr1);
    if (ret != 0) {
        return ret;
    }

    // 更新ADCDL位
    if (enable) {
        adcr1 |= BH45B1225_ADCR1_ADCDL;
    } else {
        adcr1 &= ~BH45B1225_ADCR1_ADCDL;
    }

    return dev->write_reg(dev, BH45B1225_REG_ADCR1, adcr1);
}

int bh45b1225_clear_eoc(bh45b1225_dev_t *dev)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前ADCR1配置
    uint8_t adcr1;
    int ret = dev->read_reg(dev, BH45B1225_REG_ADCR1, &adcr1);
    if (ret != 0) {
        return ret;
    }

    // 清除EOC位 (写0清除)
    adcr1 &= ~BH45B1225_ADCR1_EOC;

    return dev->write_reg(dev, BH45B1225_REG_ADCR1, adcr1);
}

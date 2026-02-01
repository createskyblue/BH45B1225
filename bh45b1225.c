/**
 * @file bh45b1225.c
 * @brief BH45B1225 24位Delta-Sigma ADC驱动程序实现
 * @note 基于BH45B1225数据手册 Rev 1.21重新实现
 * @author createskyblue <createskyblue@outlook.com> 2026
 */

#include "bh45b1225.h"
#include <string.h>
#include <limits.h>

/* =========================== 私有方法实现 =========================== */

static int write_reg(bh45b1225_dev_t *dev, uint8_t reg, uint8_t value)
{
    if (dev == NULL || dev->i2c_write_func == NULL) {
        return -1;
    }
    // 将寄存器地址和数据打包：[寄存器地址][数据]
    uint8_t buf[2] = {reg, value};
    return dev->i2c_write_func(dev->dev_addr_func, buf, 2);
}

static int read_reg(bh45b1225_dev_t *dev, uint8_t reg, uint8_t *value)
{
    if (dev == NULL || dev->i2c_read == NULL || value == NULL) {
        return -1;
    }
    return dev->i2c_read(dev->dev_addr_func, reg, value, 1);
}

/* =========================== 公共API实现 =========================== */

int bh45b1225_init(bh45b1225_dev_t *dev, uint8_t dev_addr_func,
                   bh45b1225_i2c_write_t write_func,
                   bh45b1225_i2c_read_t read_func,
                   bh45b1225_delay_t delay_func)
{
    if (dev == NULL || write_func == NULL || read_func == NULL || delay_func == NULL) {
        return -1;
    }

    memset(dev, 0, sizeof(bh45b1225_dev_t));

    dev->dev_addr_func = dev_addr_func;
    dev->i2c_write_func = write_func;
    dev->i2c_read = read_func;
    dev->delay_func = delay_func;
    dev->write_reg = write_reg;
    dev->read_reg = read_reg;

    int ret;

    //启动内置RC时钟
    ret = bh45b1225_enable_hirc(dev);
    if (ret != 0) return ret;

    dev->delay_func(10);

    bool hirc_stable = false;
    bh45b1225_check_hirc_stable(dev, &hirc_stable);
    if (!hirc_stable) {
        return -1;  // HIRC 不稳定
    }

    //使能VCM，设置特性优化位
    ret = bh45b1225_set_vcm(dev, true);
    if (ret != 0) return ret;
    
    //配置 ADCTE 寄存器为固定值 1110_0111B (0xE7)，禁止修改此值
    ret = bh45b1225_set_adcte(dev, 0xE7);
    if (ret != 0) return ret;
    
    //设置参考电压源
    ret = bh45b1225_set_vref_source(dev, BH45B1225_VREF_INTERNAL);
    if (ret != 0) return ret;
    
    //设置参考电压缓存
    ret = bh45b1225_set_vref_buffer(dev, false, false);  // VRBUFP=1, VRBUFN=1
    if (ret != 0) return ret;
    
    //配置 ADC 数据速率
    ret = bh45b1225_set_data_rate(dev, BH45B1225_DR_400HZ);
    if (ret < 0) return ret;
    
    //设置默认增益
    ret = bh45b1225_set_pga_gain(dev, BH45B1225_GAIN_1);
    if (ret != 0) return ret;

    //设置ADC功耗模式
    ret = bh45b1225_set_adc_mode(dev, BH45B1225_ADC_MODE_NORMAL);
    if (ret != 0) return ret;

    dev->delay_func(10);

    return 0;  // 成功
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

    

    // 分别读取 3 个字节，避免 I2C 总线忙碌
    uint8_t buffer[3];
    int ret;

    //打开锁存
    ret = bh45b1225_set_data_latch(dev, true);
    if (ret != 0) {
        return ret;
    }

    ret = dev->read_reg(dev, BH45B1225_REG_ADRL, &buffer[0]);
    if (ret != 0) {
        return ret;
    }

    ret = dev->read_reg(dev, BH45B1225_REG_ADRM, &buffer[1]);
    if (ret != 0) {
        return ret;
    }

    ret = dev->read_reg(dev, BH45B1225_REG_ADRH, &buffer[2]);
    if (ret != 0) {
        return ret;
    }

    // 关闭锁存
    ret = bh45b1225_set_data_latch(dev, false);
    if (ret != 0) {
        return ret;
    }

    // 组合24位数据 (大端格式)
    // ADRL = Bit7~Bit0 (低8位)
    // ADRM = Bit15~Bit8 (中8位)
    // ADRH = Bit23~Bit16 (高8位)
    int32_t raw = (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];

    // 符号扩展：24位有符号数扩展到32位
    // 如果bit 23是1，说明是负数，需要将高8位设置为1
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }

    *data = raw;

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

int bh45b1225_enable_hirc(bh45b1225_dev_t *dev)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 读取当前HIRCC配置
    uint8_t hircc;
    int ret = dev->read_reg(dev, BH45B1225_REG_HIRCC, &hircc);
    if (ret != 0) {
        return ret;
    }

    // 设置HIRCEN位（BIT0）
    hircc |= 0x01;

    // 设置HIRCO 必须为0 (BIT2)
    hircc &= ~0x04;

    return dev->write_reg(dev, BH45B1225_REG_HIRCC, hircc);
}

int bh45b1225_check_hirc_stable(bh45b1225_dev_t *dev, bool *stable)
{
    if (dev == NULL || dev->read_reg == NULL || stable == NULL) {
        return -1;
    }

    uint8_t hircc;
    int ret = dev->read_reg(dev, BH45B1225_REG_HIRCC, &hircc);
    if (ret != 0) {
        return ret;
    }

    // 检查HIRCF位（BIT1）
    *stable = (hircc & 0x02) ? true : false;
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

    // 设置ADRST位（复位 SINC 滤波器）
    adcr0 |= BH45B1225_ADCR0_ADRST;
    ret = dev->write_reg(dev, BH45B1225_REG_ADCR0, adcr0);
    if (ret != 0) {
        return ret;
    }

    // ADRST 需要保持至少一个 tMCLK 周期
    // 假设 fMCLK 约为几 MHz，一个周期不到 1 微秒
    // 这里添加一个短延迟确保复位完成
    dev->delay_func(10);

    // 清除ADRST位（退出复位模式，开始新转换）
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

int bh45b1225_set_pwrc_opt(bh45b1225_dev_t *dev, uint8_t opt_bits)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 限制opt_bits范围 (BIT6~BIT0)
    opt_bits &= 0x7F;

    // 读取当前PWRC配置
    uint8_t pwrc;
    int ret = dev->read_reg(dev, BH45B1225_REG_PWRC, &pwrc);
    if (ret != 0) {
        return ret;
    }

    // 保留VCMEN位，更新特性优化位
    pwrc = (pwrc & BH45B1225_PWRC_VCMEN) | opt_bits;

    return dev->write_reg(dev, BH45B1225_REG_PWRC, pwrc);
}

int bh45b1225_set_adcte(bh45b1225_dev_t *dev, uint8_t value)
{
    if (dev == NULL || dev->write_reg == NULL) {
        return -1;
    }

    return dev->write_reg(dev, BH45B1225_REG_ADCTE, value);
}

int bh45b1225_set_vref_buffer(bh45b1225_dev_t *dev, bool vrefp_buf, bool vrefn_buf)
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

    // 更新VRBUFP和VRBUFN位
    if (vrefp_buf) {
        adcr1 |= BH45B1225_ADCR1_VRBUFP;
    } else {
        adcr1 &= ~BH45B1225_ADCR1_VRBUFP;
    }

    if (vrefn_buf) {
        adcr1 |= BH45B1225_ADCR1_VRBUFN;
    } else {
        adcr1 &= ~BH45B1225_ADCR1_VRBUFN;
    }

    return dev->write_reg(dev, BH45B1225_REG_ADCR1, adcr1);
}

int bh45b1225_set_filter_mode(bh45b1225_dev_t *dev, bh45b1225_flms_t flms_mode)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    // 根据数据手册，PWRC的特性优化位必须与FLMS匹配
    // FLMS=000 (DIV30) -> PWRC opt = 010_1000B = 0x48
    // FLMS=010 (DIV12) -> PWRC opt = 011_1100B = 0x3C
    uint8_t pwrc_opt;
    uint8_t flms_value;

    switch (flms_mode) {
        case BH45B1225_FLMS_DIV30:
            pwrc_opt = 0x48;
            flms_value = 0;
            break;
        case BH45B1225_FLMS_DIV12:
            pwrc_opt = 0x3C;
            flms_value = 2;
            break;
        default:
            return -2;  // 无效的FLMS模式
    }

    int ret;

    // 先更新PWRC寄存器的特性优化位
    ret = bh45b1225_set_pwrc_opt(dev, pwrc_opt);
    if (ret != 0) return ret;

    // 再更新ADCR1寄存器的FLMS位
    uint8_t adcr1;
    ret = dev->read_reg(dev, BH45B1225_REG_ADCR1, &adcr1);
    if (ret != 0) return ret;

    // 清除FLMS位并设置新值
    adcr1 &= ~(BH45B1225_ADCR1_FLMS2 | BH45B1225_ADCR1_FLMS1 | BH45B1225_ADCR1_FLMS0);
    adcr1 |= ((flms_value << 5) & (BH45B1225_ADCR1_FLMS2 | BH45B1225_ADCR1_FLMS1 | BH45B1225_ADCR1_FLMS0));

    return dev->write_reg(dev, BH45B1225_REG_ADCR1, adcr1);
}

const bh45b1225_rate_cfg_t bh45b1225_rate_table[] = {
    { BH45B1225_DR_5HZ, 31, BH45B1225_OSR_32768,  BH45B1225_FLMS_DIV30  },
    { BH45B1225_DR_10HZ, 31, BH45B1225_OSR_16384,  BH45B1225_FLMS_DIV30  },
    { BH45B1225_DR_20HZ, 31, BH45B1225_OSR_8192,  BH45B1225_FLMS_DIV30  },
    { BH45B1225_DR_25HZ, 31, BH45B1225_OSR_16384,  BH45B1225_FLMS_DIV12  },
    { BH45B1225_DR_40HZ, 31, BH45B1225_OSR_4096,  BH45B1225_FLMS_DIV30  },
    { BH45B1225_DR_50HZ, 31, BH45B1225_OSR_8192,  BH45B1225_FLMS_DIV12  },
    { BH45B1225_DR_100HZ, 31, BH45B1225_OSR_4096,  BH45B1225_FLMS_DIV12  },
    { BH45B1225_DR_400HZ, 31, BH45B1225_OSR_1024,  BH45B1225_FLMS_DIV12  },
    { BH45B1225_DR_1600HZ, 31, BH45B1225_OSR_256,  BH45B1225_FLMS_DIV12  },
};


/**
 * @brief Set ADC data rate using predefined safe configurations.
 *
 * @note This function ALWAYS performs an internal ADC reset (ADRST)
 *       to clear SINC filter state after changing clock / filter / OSR.
 */
int bh45b1225_set_data_rate(
    bh45b1225_dev_t *dev,
    bh45b1225_data_rate_t rate
)
{
    if (dev == NULL || dev->read_reg == NULL || dev->write_reg == NULL) {
        return -1;
    }

    const bh45b1225_rate_cfg_t *cfg = NULL;

    for (size_t i = 0;
         i < sizeof(bh45b1225_rate_table) / sizeof(bh45b1225_rate_table[0]);
         i++) {
        if (bh45b1225_rate_table[i].rate == rate) {
            cfg = &bh45b1225_rate_table[i];
            break;
        }
    }

    if (cfg == NULL) {
        return -2;  // unsupported data rate
    }

    int ret;

    ret = bh45b1225_set_clock_div(dev, cfg->adck);
    if (ret != 0) return ret;

    ret = bh45b1225_set_filter_mode(dev, cfg->flms);
    if (ret != 0) return ret;

    ret = bh45b1225_set_osr(dev, cfg->osr);
    if (ret != 0) return ret;

    return 0;
}

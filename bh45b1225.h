/**
 * @file bh45b1225.h
 * @brief BH45B1225 24位Delta-Sigma ADC驱动程序 - 面向对象风格设计
 * @note 基于BH45B1225数据手册 Rev 1.21重新实现
 */

#ifndef BH45B1225_H
#define BH45B1225_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================== 寄存器地址定义 =========================== */
#define BH45B1225_REG_PWRC           0x00    // 电源控制寄存器
#define BH45B1225_REG_PGAC0          0x01    // PGA配置寄存器0 (增益设置)
#define BH45B1225_REG_PGAC1          0x02    // PGA配置寄存器1 (输入配置)
#define BH45B1225_REG_PGACS          0x03    // PGA配置寄存器 (通道选择)
#define BH45B1225_REG_ADRL           0x04    // ADC数据低字节
#define BH45B1225_REG_ADRM           0x05    // ADC数据中字节
#define BH45B1225_REG_ADRH           0x06    // ADC数据高字节
#define BH45B1225_REG_ADCR0          0x07    // ADC控制寄存器0
#define BH45B1225_REG_ADCR1          0x08    // ADC控制寄存器1
#define BH45B1225_REG_ADCS           0x09    // ADC采样配置
#define BH45B1225_REG_ADCTE          0x0A    // ADC测试配置

/* =========================== PWRC寄存器 (0x00) 位定义 =========================== */
#define BH45B1225_PWRC_VCMEN         0x80    // BIT7: VCM使能

/* =========================== PGAC0寄存器 (0x01) 位定义 =========================== */
// VGS1~VGS0: 参考电压增益选择 (BIT6~BIT5)
#define BH45B1225_VGS_MASK           0x60
#define BH45B1225_VGS_SHIFT          5
typedef enum {
    BH45B1225_VREFGN_1   = 0,       // VREFGN = 1
    BH45B1225_VREFGN_1_2 = 1,       // VREFGN = 1/2
    BH45B1225_VREFGN_1_4 = 2        // VREFGN = 1/4
} bh45b1225_vrefgn_t;

// AGS1~AGS0: ADC输入增益选择 (BIT4~BIT3)
#define BH45B1225_AGS_MASK           0x18
#define BH45B1225_AGS_SHIFT          3
typedef enum {
    BH45B1225_ADGN_1 = 0,          // ADGN = 1
    BH45B1225_ADGN_2 = 1            // ADGN = 2
} bh45b1225_adgn_t;

// PGS2~PGS0: PGA输入增益选择 (BIT2~BIT0)
#define BH45B1225_PGS_MASK           0x07
#define BH45B1225_PGS_SHIFT          0
typedef enum {
    BH45B1225_PGAGN_1  = 0,         // PGAGN = 1
    BH45B1225_PGAGN_2  = 1,         // PGAGN = 2
    BH45B1225_PGAGN_4  = 2,         // PGAGN = 4
    BH45B1225_PGAGN_8  = 3,         // PGAGN = 8
    BH45B1225_PGAGN_16 = 4,         // PGAGN = 16
    BH45B1225_PGAGN_32 = 5,         // PGAGN = 32
    BH45B1225_PGAGN_64 = 6          // PGAGN = 64
} bh45b1225_pgagn_t;

/* =========================== PGAC1寄存器 (0x02) 位定义 =========================== */
// INIS: 输入端内部连接控制 (BIT6)
#define BH45B1225_INIS               0x40    // BIT6: INIS - IN1/IN2内部连接

// INX1~INX0: 输入端连接控制 (BIT5~BIT4)
#define BH45B1225_INX1_MASK          0x20    // BIT5: INX1
#define BH45B1225_INX0_MASK          0x10    // BIT4: INX0

// DCSET2~DCSET0: 差分输入偏置选择 (BIT3~BIT1)
#define BH45B1225_DCSET_MASK         0x0E
#define BH45B1225_DCSET_SHIFT        1

/* =========================== PGACS寄存器 (0x03) 位定义 =========================== */
// CHSN2~CHSN0: 反相输入端IN2选择 (BIT5~BIT3)
#define BH45B1225_CHSN_MASK          0x38
#define BH45B1225_CHSN_SHIFT         3
typedef enum {
    BH45B1225_IN2_AN1   = 0,        // AN1
    BH45B1225_IN2_AN3   = 1,        // AN3
    BH45B1225_IN2_VDACO = 4,       // VDACO
    BH45B1225_IN2_AVSS   = 5,       // AVSS
    BH45B1225_IN2_VCM    = 6,       // VCM
    BH45B1225_IN2_VTSN   = 7         // VTSN (温度传感器)
} bh45b1225_in2_select_t;

// CHSP2~CHSP0: 正相输入端IN1选择 (BIT2~BIT0)
#define BH45B1225_CHSP_MASK          0x07
#define BH45B1225_CHSP_SHIFT         0
typedef enum {
    BH45B1225_IN1_AN0   = 0,        // AN0
    BH45B1225_IN1_AN2   = 1,        // AN2
    BH45B1225_IN1_VDACO = 4,        // VDACO
    BH45B1225_IN1_VCM    = 6,        // VCM
    BH45B1225_IN1_VTSP   = 7         // VTSP (温度传感器)
} bh45b1225_in1_select_t;

/* =========================== ADCR0寄存器 (0x07) 位定义 =========================== */
#define BH45B1225_ADCR0_ADRST        0x80    // BIT7: ADC软件复位
#define BH45B1225_ADCR0_ADSL        0x40    // BIT6: 休眠模式 (手册中称为ADSLP)
#define BH45B1225_ADCR0_ADOFF        0x20    // BIT5: ADC电源控制
#define BH45B1225_ADCR0_ADOR_MASK    0x1E    // BIT4~BIT1: 过采样率选择
#define BH45B1225_ADCR0_ADOR_SHIFT   1
#define BH45B1225_ADCR0_VREFS       0x01    // BIT0: 参考电压选择 (0=内部, 1=外部)

/* =========================== ADCR1寄存器 (0x08) 位定义 =========================== */
#define BH45B1225_ADCR1_EOC          0x02    // BIT1: 转换结束标志
#define BH45B1225_ADCR1_VRBUFP      0x10    // BIT4: 正参考电压缓存
#define BH45B1225_ADCR1_VRBUFN      0x08    // BIT3: 负参考电压缓存
#define BH45B1225_ADCR1_ADCDL       0x04    // BIT2: 数据锁存

/* =========================== ADCS寄存器 (0x09) 位定义 =========================== */
#define BH45B1225_ADCS_ADCK_MASK     0x1F    // BIT4~BIT0: ADC时钟分频选择

/* =========================== DAC寄存器定义 =========================== */
#define BH45B1225_REG_DAH            0x0B    // DAC高字节寄存器
#define BH45B1225_REG_DAL            0x0C    // DAC低字节寄存器
#define BH45B1225_REG_DACC           0x0D    // DAC控制寄存器

// DACC寄存器位定义
#define BH45B1225_DACC_DACEN         0x80    // BIT7: DAC使能
#define BH45B1225_DACC_DACVRS        0x40    // BIT6: DAC参考电压选择 (0=AVDD, 1=VCM)

/* =========================== 类型定义 =========================== */

/**
 * @brief I2C读写函数指针类型 - 用于硬件解耦
 */
typedef int (*bh45b1225_i2c_write_t)(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);
typedef int (*bh45b1225_i2c_read_t)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief 模拟输入通道
 */
typedef enum {
    BH45B1225_AN0 = 0,
    BH45B1225_AN1,
    BH45B1225_AN2,
    BH45B1225_AN3
} bh45b1225_analog_input_t;

/**
 * @brief PGA总增益配置
 */
typedef enum {
    BH45B1225_GAIN_1   = 0,        // 总增益 = 1
    BH45B1225_GAIN_2   = 1,        // 总增益 = 2
    BH45B1225_GAIN_4   = 2,        // 总增益 = 4
    BH45B1225_GAIN_8   = 3,        // 总增益 = 8
    BH45B1225_GAIN_16  = 4,        // 总增益 = 16
    BH45B1225_GAIN_32  = 5,        // 总增益 = 32
    BH45B1225_GAIN_64  = 6,        // 总增益 = 64
    BH45B1225_GAIN_128 = 7         // 总增益 = 128
} bh45b1225_total_gain_t;

/**
 * @brief ADC过采样率(OSR)选择
 * @note 数据传输率 = f_MCLK / (N × OSR)，其中N由FLMS位选择(30或12)
 */
typedef enum {
    BH45B1225_OSR_32768 = 0,       // ADOR[3:0]=0000, OSR = 32768
    BH45B1225_OSR_16384 = 1,       // ADOR[3:0]=0001, OSR = 16384
    BH45B1225_OSR_8192  = 2,       // ADOR[3:0]=0010, OSR = 8192
    BH45B1225_OSR_4096  = 3,       // ADOR[3:0]=0011, OSR = 4096
    BH45B1225_OSR_2048  = 4,       // ADOR[3:0]=0100, OSR = 2048
    BH45B1225_OSR_1024  = 5,       // ADOR[3:0]=0101, OSR = 1024
    BH45B1225_OSR_512   = 6,       // ADOR[3:0]=0110, OSR = 512
    BH45B1225_OSR_256   = 7,       // ADOR[3:0]=0111, OSR = 256
    BH45B1225_OSR_128   = 8        // ADOR[3:0]=1000, OSR = 128
} bh45b1225_osr_t;

/**
 * @brief 参考电压选择
 */
typedef enum {
    BH45B1225_VREF_INTERNAL = 0,    // 内部参考 (VCM & AVSS)
    BH45B1225_VREF_EXTERNAL = 1     // 外部参考 (VREFP & VREFN)
} bh45b1225_vref_t;

/**
 * @brief DAC参考电压选择
 */
typedef enum {
    BH45B1225_DAC_VREF_AVDD = 0,    // DAC参考电压来自AVDD
    BH45B1225_DAC_VREF_VCM  = 1     // DAC参考电压来自VCM
} bh45b1225_dac_vref_t;

/**
 * @brief ADC工作模式
 */
typedef enum {
    BH45B1225_ADC_MODE_NORMAL = 0,  // 正常模式
    BH45B1225_ADC_MODE_SLEEP  = 1,  // 休眠模式 (保留PGA和Bandgap)
    BH45B1225_ADC_MODE_POWEROFF = 2 // 掉电模式
} bh45b1225_adc_mode_t;

/**
 * @brief BH45B1225设备对象结构体
 */
typedef struct bh45b1225_dev_t {
    uint8_t dev_addr;                           // I2C设备地址
    bh45b1225_i2c_write_t i2c_write;            // I2C写函数指针
    bh45b1225_i2c_read_t  i2c_read;             // I2C读函数指针

    // 私有方法
    int (*write_reg)(struct bh45b1225_dev_t *dev, uint8_t reg, uint8_t value);
    int (*read_reg)(struct bh45b1225_dev_t *dev, uint8_t reg, uint8_t *value);
} bh45b1225_dev_t;

/* =========================== API函数声明 =========================== */

/**
 * @brief 初始化BH45B1225设备对象
 * @param dev 设备对象指针
 * @param dev_addr I2C设备地址 (7位地址)
 * @param write_func I2C写函数指针
 * @param read_func I2C读函数指针
 * @return 0=成功, <0=失败
 */
int bh45b1225_init(bh45b1225_dev_t *dev, uint8_t dev_addr,
                   bh45b1225_i2c_write_t write_func,
                   bh45b1225_i2c_read_t read_func);

/**
 * @brief 反初始化设备对象
 */
void bh45b1225_deinit(bh45b1225_dev_t *dev);

/**
 * @brief 设置PGA总增益
 * @param dev 设备对象指针
 * @param gain 总增益 (1, 2, 4, 8, 16, 32, 64, 128)
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_pga_gain(bh45b1225_dev_t *dev, bh45b1225_total_gain_t gain);

/**
 * @brief 配置输入通道
 * @param dev 设备对象指针
 * @param in1_pos 正相输入选择
 * @param in2_neg 反相输入选择
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_input_channel(bh45b1225_dev_t *dev,
                                bh45b1225_in1_select_t in1_pos,
                                bh45b1225_in2_select_t in2_neg);

/**
 * @brief 设置输入极性 (通过INX位控制)
 * @param dev 设备对象指针
 * @param inx_value INX[1:0]值 (0-3)
 * @return 0=成功, <0=失败
 * @note INX位控制IN1/IN2与DI+/DI-的连接方式
 */
int bh45b1225_set_inx_polarity(bh45b1225_dev_t *dev, uint8_t inx_value);

/**
 * @brief 使能VCM
 * @param dev 设备对象指针
 * @param enable true=使能, false=除能
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_vcm(bh45b1225_dev_t *dev, bool enable);

/**
 * @brief 启动ADC转换
 * @param dev 设备对象指针
 * @return 0=成功, <0=失败
 */
int bh45b1225_start_conversion(bh45b1225_dev_t *dev);

/**
 * @brief 读取24位ADC转换结果
 * @param dev 设备对象指针
 * @param data 存储读取数据的指针 (32位有符号值)
 * @return 0=成功, <0=失败
 */
int bh45b1225_read_data(bh45b1225_dev_t *dev, int32_t *data);

/**
 * @brief 检查ADC转换是否完成
 * @param dev 设备对象指针
 * @param complete 完成标志 (true=完成, false=进行中)
 * @return 0=成功, <0=失败
 */
int bh45b1225_check_eoc(bh45b1225_dev_t *dev, bool *complete);

/**
 * @brief 将ADC码转换为电压值
 * @param adc_code 24位ADC码 (有符号)
 * @param vref 参考电压
 * @param total_gain 总增益
 * @return 电压值 (与vref单位相同)
 */
float bh45b1225_code_to_voltage(int32_t adc_code, float vref, float total_gain);

/**
 * @brief 设置ADC过采样率(OSR)
 * @param dev 设备对象指针
 * @param osr 过采样率配置
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_osr(bh45b1225_dev_t *dev, bh45b1225_osr_t osr);

/**
 * @brief 设置ADC时钟分频
 * @param dev 设备对象指针
 * @param div_val 分频值 (0-30对应fSYS/2/1~fSYS/2/31, 31对应fSYS)
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_clock_div(bh45b1225_dev_t *dev, uint8_t div_val);

/**
 * @brief 设置参考电压源
 * @param dev 设备对象指针
 * @param vref 参考电压类型 (内部/外部)
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_vref_source(bh45b1225_dev_t *dev, bh45b1225_vref_t vref);

/**
 * @brief 设置DAC输出值
 * @param dev 设备对象指针
 * @param value 12位DAC值 (0-4095)
 * @return 0=成功, <0=失败
 * @note 必须先写DAL再写DAH才会生效
 */
int bh45b1225_set_dac_output(bh45b1225_dev_t *dev, uint16_t value);

/**
 * @brief 使能/除能DAC
 * @param dev 设备对象指针
 * @param enable true=使能, false=除能
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_dac_enable(bh45b1225_dev_t *dev, bool enable);

/**
 * @brief 设置DAC参考电压源
 * @param dev 设备对象指针
 * @param vref DAC参考电压类型
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_dac_vref(bh45b1225_dev_t *dev, bh45b1225_dac_vref_t vref);

/**
 * @brief 配置温度传感器通道
 * @param dev 设备对象指针
 * @return 0=成功, <0=失败
 * @note 将CHSP设为VTSP，CHSN设为VTSN
 */
int bh45b1225_config_temp_sensor(bh45b1225_dev_t *dev);

/**
 * @brief 读取温度传感器ADC值并转换为温度
 * @param dev 设备对象指针
 * @param temperature 存储温度值的指针 (单位: ℃)
 * @return 0=成功, <0=失败
 * @note 温度传感器系数: 175μV/℃，需先调用bh45b1225_config_temp_sensor配置通道
 */
int bh45b1225_read_temperature(bh45b1225_dev_t *dev, float *temperature);

/**
 * @brief 设置ADC工作模式
 * @param dev 设备对象指针
 * @param mode 工作模式 (正常/休眠/掉电)
 * @return 0=成功, <0=失败
 */
int bh45b1225_set_adc_mode(bh45b1225_dev_t *dev, bh45b1225_adc_mode_t mode);

/**
 * @brief 复位ADC滤波器
 * @param dev 设备对象指针
 * @return 0=成功, <0=失败
 * @note 复位后当前转换数据失效，需要重新等待转换完成
 */
int bh45b1225_reset_adc_filter(bh45b1225_dev_t *dev);

/**
 * @brief 使能/除能数据锁存
 * @param dev 设备对象指针
 * @param enable true=锁存数据, false=正常更新
 * @return 0=成功, <0=失败
 * @note 锁存后读取数据不会被更新，读取完成后建议关闭锁存
 */
int bh45b1225_set_data_latch(bh45b1225_dev_t *dev, bool enable);

/**
 * @brief 清除EOC标志
 * @param dev 设备对象指针
 * @return 0=成功, <0=失败
 */
int bh45b1225_clear_eoc(bh45b1225_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BH45B1225_H */

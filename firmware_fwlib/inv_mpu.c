/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.c
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inv_mpu.h"

/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data_)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data_)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */
#include "fw_hal.h"

#define i2c_write(addr, reg, len, data_) (I2C_Write(addr, reg, data_, len) != HAL_OK)
#define i2c_read(addr, reg, len, data_) (I2C_Read(addr, reg, data_, len) != HAL_OK)
#define delay_ms(ms)            SYS_Delay(ms)
#define get_ms(ptr)             ;
#define reg_int_cb(param)       (0)
#define log_i                   (void)
#define log_e                   (void)
#define min(a, b)               (((a) < (b)) ? (a) : (b))
static uint8_t eeprom_read_bytes(uint16_t addr, uint8_t *buf, uint8_t size) compact
{
    IAP_SetEnabled(HAL_State_ON);
    do
    {
        IAP_CmdRead(addr);
        if (IAP_IsCmdFailed())
        {
            return HAL_ERROR;
        }
        else
        {
            *buf = IAP_ReadData();
            buf++;
            addr++;
        }
    } while (--size);
    IAP_SetEnabled(HAL_State_OFF);
    return HAL_OK;
}

#if !defined MPU6050 && !defined MPU9150 && !defined MPU6500 && !defined MPU9250
#error  Which gyro are you using? Define MPUxxxx in your compiler options.
#endif

/* Time for some messy macro work. =]
 * #define MPU9150
 * is equivalent to..
 * #define MPU6050
 * #define AK8975_SECONDARY
 *
 * #define MPU9250
 * is equivalent to..
 * #define MPU6500
 * #define AK8963_SECONDARY
 */
#if defined MPU9150
#ifndef MPU6050
#define MPU6050
#endif                          /* #ifndef MPU6050 */
#if defined AK8963_SECONDARY
#error "MPU9150 and AK8963_SECONDARY cannot both be defined."
#elif !defined AK8975_SECONDARY /* #if defined AK8963_SECONDARY */
#define AK8975_SECONDARY
#endif                          /* #if defined AK8963_SECONDARY */
#elif defined MPU9250           /* #if defined MPU9150 */
#ifndef MPU6500
#define MPU6500
#endif                          /* #ifndef MPU6500 */
#if defined AK8975_SECONDARY
#error "MPU9250 and AK8975_SECONDARY cannot both be defined."
#elif !defined AK8963_SECONDARY /* #if defined AK8975_SECONDARY */
#define AK8963_SECONDARY
#endif                          /* #if defined AK8975_SECONDARY */
#endif                          /* #if defined MPU9150 */

#if defined AK8975_SECONDARY || defined AK8963_SECONDARY
#define AK89xx_SECONDARY
#else
/* #warning "No compass = less profit for Invensense. Lame." */
#endif

static int set_int_enable(unsigned char enable);

/* Hardware registers needed by driver. */
struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
#if defined AK89xx_SECONDARY
    unsigned char s0_addr;
    unsigned char s0_reg;
    unsigned char s0_ctrl;
    unsigned char s1_addr;
    unsigned char s1_reg;
    unsigned char s1_ctrl;
    unsigned char s4_ctrl;
    unsigned char s0_do;
    unsigned char s1_do;
    unsigned char i2c_delay_ctrl;
    unsigned char raw_compass;
    /* The I2C_MST_VDDIO bit is in this register. */
    unsigned char yg_offs_tc;
#endif
};

/* Information specific to a particular device. */
struct hw_s {
    unsigned char addr;
    unsigned short max_fifo;
    unsigned char num_reg;
    unsigned short temp_sens;
    short temp_offset;
    unsigned short bank_size;
#if defined AK89xx_SECONDARY
    unsigned short compass_fsr;
#endif
};

/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s {
    unsigned short gyro_fsr;
    unsigned char accel_fsr;
    unsigned short lpf;
    unsigned short sample_rate;
    unsigned char sensors_on;
    unsigned char fifo_sensors;
    unsigned char dmp_on;
};

/* Cached chip configuration data_.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    unsigned char gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    unsigned char accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    unsigned char sensors;
    /* Matches config register. */
    unsigned char lpf;
    unsigned char clk_src;
    /* Sample rate, NOT rate divider. */
    unsigned short sample_rate;
    /* Matches fifo_en register. */
    unsigned char fifo_enable;
    /* Matches int enable register. */
    unsigned char int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    unsigned char bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    unsigned char accel_half;
    /* 1 if device in low-power accel-only mode. */
    unsigned char lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    unsigned char int_motion_only;
    struct motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    unsigned char active_low_int;
    /* 1 for latched interrupts. */
    unsigned char latched_int;
    /* 1 if DMP is enabled. */
    unsigned char dmp_on;
    /* Ensures that DMP will only be loaded once. */
    unsigned char dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    unsigned short dmp_sample_rate;
#ifdef AK89xx_SECONDARY
    /* Compass sample rate. */
    unsigned short compass_sample_rate;
    unsigned char compass_addr;
    short mag_sens_adj[3];
#endif
};

/* Information for self-test. */
struct test_s {
    unsigned long gyro_sens;
    unsigned long accel_sens;
    unsigned char reg_rate_div;
    unsigned char reg_lpf;
    unsigned char reg_gyro_fsr;
    unsigned char reg_accel_fsr;
    unsigned short wait_ms;
    unsigned char packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
#ifdef MPU6500
    float max_g_offset;
    unsigned short sample_wait_ms;
#endif
};

/* Gyro driver state variables. */
struct gyro_state_s {
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
    struct chip_cfg_s chip_cfg;
    const struct test_s *test;
};

/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
#if defined MPU6050
    INV_LPA_1_25HZ,
    INV_LPA_5HZ,
    INV_LPA_20HZ,
    INV_LPA_40HZ
#elif defined MPU6500
    INV_LPA_0_24HZ,
    INV_LPA_0_49HZ,
    INV_LPA_0_98HZ,
    INV_LPA_1_95HZ,
    INV_LPA_3_91HZ,
    INV_LPA_7_81HZ,
    INV_LPA_15_63HZ,
    INV_LPA_31_25HZ,
    INV_LPA_62_50HZ,
    INV_LPA_125HZ,
    INV_LPA_250HZ,
    INV_LPA_500HZ
#endif
};

#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)
#define BIT_ACCL_FC_B       (0x08)

#if defined AK8975_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x00)
#define AK89xx_FSR                  (9830)
#elif defined AK8963_SECONDARY
#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AK89xx_FSR                  (4915)
#endif

#ifdef AK89xx_SECONDARY
#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)
#endif

#if defined MPU6050
const struct gyro_reg_s reg = {
    0x75,
    0x19,
    0x1A,
    0x0C,
    0x6A,
    0x23,
    0x1B,
    0x1C,
    0, 0, 0x1F,
    0x20,
    0x72,
    0x74,
    0x43,
    0x3B,
    0x41,
    0x38,
    0x39,
    0x3A,
    0, 0x6B,
    0x6C,
    0x37,
    0x6F,
    0x06,
    0x24,
    0x6D,
    0x6E,
    0x70
#ifdef AK89xx_SECONDARY
    ,0x25,
    0x26,
    0x27,
    0x28,
    0x29,
    0x2A,
    0x34,
    0x63,
    0x64,
    0x67,
    0x49,
    0x01
#endif
};
const struct hw_s hw = {
    0x68,
    1024,
    118,
    340,
    -521,
    256
#if defined AK89xx_SECONDARY
    ,AK89xx_FSR
#endif
};

const struct test_s test = {
    32768/250,
    32768/16,
    0,    /* 1kHz. */
    1,    /* 188Hz. */
    0,    /* 250dps. */
    0x18, /* 16g. */
    50,
    5,    /* 5% */
    10.f,
    105.f,
    0.14f,
    0.3f,
    0.95f,
    0.14f
};

static struct gyro_state_s st = {
    &reg,
    &hw,
    { 0 }, &test
};
#elif defined MPU6500
const struct gyro_reg_s reg = {
    0x75,
    0x19,
    0x1A,
    0x0C,
    0x6A,
    0x23,
    0x1B,
    0x1C,
    0x1D,
    0x1E,
    0x1F,
    0x20,
    0x72,
    0x74,
    0x43,
    0x3B,
    0x41,
    0x38,
    0x39,
    0x3A,
    0x69,
    0x6B,
    0x6C,
    0x37,
    0x6F,
    0x77,
    0x24,
    0x6D,
    0x6E,
    0x70
#ifdef AK89xx_SECONDARY
    ,0x25,
    0x26,
    0x27,
    0x28,
    0x29,
    0x2A,
    0x34,
    0x63,
    0x64,
    0x67,
    0x49
#endif
};
const struct hw_s hw = {
    0x68,
    1024,
    128,
    321,
    0,
    256
#if defined AK89xx_SECONDARY
    ,AK89xx_FSR
#endif
};

const struct test_s test = {
    32768/250,
    32768/2,  //FSR = +-2G = 16384 LSB/G
    0,    /* 1kHz. */
    2,    /* 92Hz low pass filter*/
    0,    /* 250dps. */
    0x0,  /* Accel FSR setting = 2g. */
    200,   //200ms stabilization time
    200,    /* 200 samples */
    20.f,  //20 dps for Gyro Criteria C
    60.f, //Must exceed 60 dps threshold for Gyro Criteria B
    .5f, //Must exceed +50% variation for Gyro Criteria A
    .225f, //Accel must exceed Min 225 mg for Criteria B
    .675f, //Accel cannot exceed Max 675 mg for Criteria B
    .5f,  //Accel must be within 50% variation for Criteria A
    .5f,   //500 mg for Accel Criteria C
    10    //10ms sample time wait
};

static struct gyro_state_s st = {
    &reg,
    &hw,
    { 0 }, &test
};
#endif

#define MAX_PACKET_LENGTH (12)
#ifdef MPU6500
#define HWST_MAX_PACKET_LENGTH (512)
#endif

#ifdef AK89xx_SECONDARY
static int setup_compass(void);
#define MAX_COMPASS_SAMPLE_RATE (100)
#endif

/**
 *  @brief      Enable/disable data_ ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data_ ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
static int set_int_enable(unsigned char enable)
{
    unsigned char tmp;

    if (st.chip_cfg.dmp_on) {
        if (enable)
            tmp = BIT_DMP_INT_EN;
        else
            tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
            return -1;
        st.chip_cfg.int_enable = tmp;
    } else {
        if (!st.chip_cfg.sensors)
            return -1;
        if (enable && st.chip_cfg.int_enable)
            return 0;
        if (enable)
            tmp = BIT_DATA_RDY_EN;
        else
            tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp))
            return -1;
        st.chip_cfg.int_enable = tmp;
    }
    return 0;
}

/**
 *  @brief      Register dump for testing.
 *  @return     0 if successful.
 */
int mpu_reg_dump(void)
{
    unsigned char ii;
    unsigned char data_;

    for (ii = 0; ii < st.hw->num_reg; ii++) {
        if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w)
            continue;
        if (i2c_read(st.hw->addr, ii, 1, &data_))
            return -1;
        log_i("%#5x: %#5x\r\n", ii, data_);
    }
    return 0;
}

/**
 *  @brief      Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data_    Register data_.
 *  @return     0 if successful.
 */
int mpu_read_reg(unsigned char reg, unsigned char *data_)
{
    if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w)
        return -1;
    if (reg >= st.hw->num_reg)
        return -1;
    return i2c_read(st.hw->addr, reg, 1, data_);
}

/**
 *  @brief      Initialize hardware.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
int mpu_init(struct int_param_s *int_param)
{
    unsigned char data_[6];

    /* Reset device. */
    data_[0] = BIT_RESET;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data_))
        return -1;
    delay_ms(100);

    /* Wake up chip. */
    data_[0] = 0x00;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data_))
        return -1;

   st.chip_cfg.accel_half = 0;

#ifdef MPU6500
    /* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
     * first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
     */
    data_[0] = BIT_FIFO_SIZE_1024;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, data_))
        return -1;
#endif

    /* Set to invalid values to ensure no I2C writes are skipped. */
    st.chip_cfg.sensors = 0xFF;
    st.chip_cfg.gyro_fsr = 0xFF;
    st.chip_cfg.accel_fsr = 0xFF;
    st.chip_cfg.lpf = 0xFF;
    st.chip_cfg.sample_rate = 0xFFFF;
    st.chip_cfg.fifo_enable = 0xFF;
    st.chip_cfg.bypass_mode = 0xFF;
#ifdef AK89xx_SECONDARY
    st.chip_cfg.compass_sample_rate = 0xFFFF;
#endif
    /* mpu_set_sensors always preserves this setting. */
    st.chip_cfg.clk_src = INV_CLK_PLL;
    /* Handled in next call to mpu_set_bypass. */
    st.chip_cfg.active_low_int = 1;
    st.chip_cfg.latched_int = 0;
    st.chip_cfg.int_motion_only = 0;
    st.chip_cfg.lp_accel_mode = 0;
    memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
    st.chip_cfg.dmp_on = 0;
    st.chip_cfg.dmp_loaded = 0;
    st.chip_cfg.dmp_sample_rate = 0;

    if (mpu_set_gyro_fsr(2000))
        return -1;
    if (mpu_set_accel_fsr(2))
        return -1;
    if (mpu_set_lpf(42))
        return -1;
    if (mpu_set_sample_rate(50))
        return -1;
    if (mpu_configure_fifo(0))
        return -1;

    if (int_param)
        reg_int_cb(int_param);

#ifdef AK89xx_SECONDARY
    setup_compass();
    if (mpu_set_compass_sample_rate(10))
        return -1;
#else
    /* Already disabled by setup_compass. */
    if (mpu_set_bypass(0))
        return -1;
#endif

    mpu_set_sensors(0);
    return 0;
}

/**
 *  @brief      Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  \n MPU6500: 0.24Hz, 0.49Hz, 0.98Hz, 1.95Hz, 3.91Hz, 7.81Hz, 15.63Hz, 31.25Hz, 62.5Hz, 125Hz, 250Hz, 500Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
int mpu_lp_accel_mode(unsigned short rate)
{
    unsigned char tmp[2];

#if defined MPU6500
    unsigned char data_;
#endif

    if (!rate) {
        mpu_set_int_latched(0);
        tmp[0] = 0;
        tmp[1] = BIT_STBY_XYZG;
        if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
            return -1;
        st.chip_cfg.lp_accel_mode = 0;
        return 0;
    }
    /* For LP accel, we automatically configure the hardware to produce latched
     * interrupts. In LP accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the MCU.
     *
     * Any register read will clear the interrupt.
     */
    mpu_set_int_latched(1);
#if defined MPU6050
    tmp[0] = BIT_LPA_CYCLE;
    if (rate == 1) {
        tmp[1] = INV_LPA_1_25HZ;
        mpu_set_lpf(5);
    } else if (rate <= 5) {
        tmp[1] = INV_LPA_5HZ;
        mpu_set_lpf(5);
    } else if (rate <= 20) {
        tmp[1] = INV_LPA_20HZ;
        mpu_set_lpf(10);
    } else {
        tmp[1] = INV_LPA_40HZ;
        mpu_set_lpf(20);
    }
    tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp))
        return -1;
#elif defined MPU6500
    /* Set wake frequency. */
    if (rate == 1)
    	data_ = INV_LPA_0_98HZ;
    else if (rate == 2)
    	data_ = INV_LPA_1_95HZ;
    else if (rate <= 5)
    	data_ = INV_LPA_3_91HZ;
    else if (rate <= 10)
    	data_ = INV_LPA_7_81HZ;
    else if (rate <= 20)
    	data_ = INV_LPA_15_63HZ;
    else if (rate <= 40)
    	data_ = INV_LPA_31_25HZ;
    else if (rate <= 70)
    	data_ = INV_LPA_62_50HZ;
    else if (rate <= 125)
    	data_ = INV_LPA_125HZ;
    else if (rate <= 250)
    	data_ = INV_LPA_250HZ;
    else
    	data_ = INV_LPA_500HZ;
        
    if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, &data_))
        return -1;
    
    if (i2c_read(st.hw->addr, st.reg->accel_cfg2, 1, &data_))
        return -1;
        
    data_ = data_ | BIT_ACCL_FC_B;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, &data_))
            return -1;
            
    data_ = BIT_LPA_CYCLE;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data_))
        return -1;
#endif
    st.chip_cfg.sensors = INV_XYZ_ACCEL;
    st.chip_cfg.clk_src = 0;
    st.chip_cfg.lp_accel_mode = 1;
    mpu_configure_fifo(0);

    return 0;
}

/**
 *  @brief      Read raw gyro data_ directly from the registers.
 *  @param[out] data_        Raw data_ in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_gyro_reg(short *data_, unsigned long *timestamp)
{
    unsigned char tmp[6];

    if (!(st.chip_cfg.sensors & INV_XYZ_GYRO))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->raw_gyro, 6, tmp))
        return -1;
    data_[0] = (tmp[0] << 8) | tmp[1];
    data_[1] = (tmp[2] << 8) | tmp[3];
    data_[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp)
        get_ms(timestamp);
    return 0;
}

/**
 *  @brief      Read raw accel data_ directly from the registers.
 *  @param[out] data_        Raw data_ in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_accel_reg(short *data_, unsigned long *timestamp)
{
    unsigned char tmp[6];

    if (!(st.chip_cfg.sensors & INV_XYZ_ACCEL))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->raw_accel, 6, tmp))
        return -1;
    data_[0] = (tmp[0] << 8) | tmp[1];
    data_[1] = (tmp[2] << 8) | tmp[3];
    data_[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp)
        get_ms(timestamp);
    return 0;
}

/**
 *  @brief      Read temperature data_ directly from the registers.
 *  @param[out] data_        Data in q16 format.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_temperature(long *data_, unsigned long *timestamp)
{
    unsigned char tmp[2];
    short raw;

    if (!(st.chip_cfg.sensors))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->temp, 2, tmp))
        return -1;
    raw = (tmp[0] << 8) | tmp[1];
    if (timestamp)
        get_ms(timestamp);

    data_[0] = (long)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536L);
    return 0;
}

/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int mpu_reset_fifo(void)
{
    unsigned char data_;

    if (!(st.chip_cfg.sensors))
        return -1;

    data_ = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data_))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data_))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data_))
        return -1;

    if (st.chip_cfg.dmp_on) {
        data_ = BIT_FIFO_RST | BIT_DMP_RST;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data_))
            return -1;
        delay_ms(50);
        data_ = BIT_DMP_EN | BIT_FIFO_EN;
        if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
            data_ |= BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data_))
            return -1;
        if (st.chip_cfg.int_enable)
            data_ = BIT_DMP_INT_EN;
        else
            data_ = 0;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data_))
            return -1;
        data_ = 0;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data_))
            return -1;
    } else {
        data_ = BIT_FIFO_RST;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data_))
            return -1;
        if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors & INV_XYZ_COMPASS))
            data_ = BIT_FIFO_EN;
        else
            data_ = BIT_FIFO_EN | BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data_))
            return -1;
        delay_ms(50);
        if (st.chip_cfg.int_enable)
            data_ = BIT_DATA_RDY_EN;
        else
            data_ = 0;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data_))
            return -1;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable))
            return -1;
    }
    return 0;
}

/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_gyro_fsr(unsigned short *fsr)
{
    switch (st.chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS:
        fsr[0] = 250;
        break;
    case INV_FSR_500DPS:
        fsr[0] = 500;
        break;
    case INV_FSR_1000DPS:
        fsr[0] = 1000;
        break;
    case INV_FSR_2000DPS:
        fsr[0] = 2000;
        break;
    default:
        fsr[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_gyro_fsr(unsigned short fsr)
{
    unsigned char data_;

    if (!(st.chip_cfg.sensors))
        return -1;

    switch (fsr) {
    case 250:
        data_ = INV_FSR_250DPS << 3;
        break;
    case 500:
        data_ = INV_FSR_500DPS << 3;
        break;
    case 1000:
        data_ = INV_FSR_1000DPS << 3;
        break;
    case 2000:
        data_ = INV_FSR_2000DPS << 3;
        break;
    default:
        return -1;
    }

    if (st.chip_cfg.gyro_fsr == (data_ >> 3))
        return 0;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data_))
        return -1;
    st.chip_cfg.gyro_fsr = data_ >> 3;
    return 0;
}

/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_accel_fsr(unsigned char *fsr)
{
    switch (st.chip_cfg.accel_fsr) {
    case INV_FSR_2G:
        fsr[0] = 2;
        break;
    case INV_FSR_4G:
        fsr[0] = 4;
        break;
    case INV_FSR_8G:
        fsr[0] = 8;
        break;
    case INV_FSR_16G:
        fsr[0] = 16;
        break;
    default:
        return -1;
    }
    if (st.chip_cfg.accel_half)
        fsr[0] <<= 1;
    return 0;
}

/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_accel_fsr(unsigned char fsr)
{
    unsigned char data_;

    if (!(st.chip_cfg.sensors))
        return -1;

    switch (fsr) {
    case 2:
        data_ = INV_FSR_2G << 3;
        break;
    case 4:
        data_ = INV_FSR_4G << 3;
        break;
    case 8:
        data_ = INV_FSR_8G << 3;
        break;
    case 16:
        data_ = INV_FSR_16G << 3;
        break;
    default:
        return -1;
    }

    if (st.chip_cfg.accel_fsr == (data_ >> 3))
        return 0;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data_))
        return -1;
    st.chip_cfg.accel_fsr = data_ >> 3;
    return 0;
}

/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int mpu_get_lpf(unsigned short *lpf)
{
    switch (st.chip_cfg.lpf) {
    case INV_FILTER_188HZ:
        lpf[0] = 188;
        break;
    case INV_FILTER_98HZ:
        lpf[0] = 98;
        break;
    case INV_FILTER_42HZ:
        lpf[0] = 42;
        break;
    case INV_FILTER_20HZ:
        lpf[0] = 20;
        break;
    case INV_FILTER_10HZ:
        lpf[0] = 10;
        break;
    case INV_FILTER_5HZ:
        lpf[0] = 5;
        break;
    case INV_FILTER_256HZ_NOLPF2:
    case INV_FILTER_2100HZ_NOLPF:
    default:
        lpf[0] = 0;
        break;
    }
    return 0;
}

/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int mpu_set_lpf(unsigned short lpf)
{
    unsigned char data_;

    if (!(st.chip_cfg.sensors))
        return -1;

    if (lpf >= 188)
        data_ = INV_FILTER_188HZ;
    else if (lpf >= 98)
        data_ = INV_FILTER_98HZ;
    else if (lpf >= 42)
        data_ = INV_FILTER_42HZ;
    else if (lpf >= 20)
        data_ = INV_FILTER_20HZ;
    else if (lpf >= 10)
        data_ = INV_FILTER_10HZ;
    else
        data_ = INV_FILTER_5HZ;

    if (st.chip_cfg.lpf == data_)
        return 0;

    if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data_))
        return -1;

#ifdef MPU6500 //MPU6500 accel/gyro dlpf separately
    data_ = BIT_FIFO_SIZE_1024 | data_;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, &data_))
            return -1;
#endif

    st.chip_cfg.lpf = data_;
    return 0;
}

/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_sample_rate(unsigned short *rate)
{
    if (st.chip_cfg.dmp_on)
        return -1;
    else
        rate[0] = st.chip_cfg.sample_rate;
    return 0;
}

/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_sample_rate(unsigned short rate)
{
    unsigned char data_;

    if (!(st.chip_cfg.sensors))
        return -1;

    if (st.chip_cfg.dmp_on)
        return -1;
    else {
        if (st.chip_cfg.lp_accel_mode) {
            if (rate && (rate <= 40)) {
                /* Just stay in low-power accel mode. */
                mpu_lp_accel_mode(rate);
                return 0;
            }
            /* Requested rate exceeds the allowed frequencies in LP accel mode,
             * switch back to full-power mode.
             */
            mpu_lp_accel_mode(0);
        }
        if (rate < 4)
            rate = 4;
        else if (rate > 1000)
            rate = 1000;

        data_ = 1000 / rate - 1;
        if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data_))
            return -1;

        st.chip_cfg.sample_rate = 1000 / (1 + data_);

#ifdef AK89xx_SECONDARY
        mpu_set_compass_sample_rate(min(st.chip_cfg.compass_sample_rate, MAX_COMPASS_SAMPLE_RATE));
#endif

        /* Automatically set LPF to 1/2 sampling rate. */
        mpu_set_lpf(st.chip_cfg.sample_rate >> 1);
        return 0;
    }
}

/**
 *  @brief      Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int mpu_get_gyro_sens(float *sens)
{
    switch (st.chip_cfg.gyro_fsr) {
    case INV_FSR_250DPS:
        sens[0] = 131.f;
        break;
    case INV_FSR_500DPS:
        sens[0] = 65.5f;
        break;
    case INV_FSR_1000DPS:
        sens[0] = 32.8f;
        break;
    case INV_FSR_2000DPS:
        sens[0] = 16.4f;
        break;
    default:
        return -1;
    }
    return 0;
}

/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int mpu_get_accel_sens(unsigned short *sens)
{
    switch (st.chip_cfg.accel_fsr) {
    case INV_FSR_2G:
        sens[0] = 16384;
        break;
    case INV_FSR_4G:
        sens[0] = 8092;
        break;
    case INV_FSR_8G:
        sens[0] = 4096;
        break;
    case INV_FSR_16G:
        sens[0] = 2048;
        break;
    default:
        return -1;
    }
    if (st.chip_cfg.accel_half)
        sens[0] >>= 1;
    return 0;
}

/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int mpu_get_fifo_config(unsigned char *sensors)
{
    sensors[0] = st.chip_cfg.fifo_enable;
    return 0;
}

/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int mpu_configure_fifo(unsigned char sensors)
{
    unsigned char prev;
    int result = 0;

    /* Compass data_ isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS;

    if (st.chip_cfg.dmp_on)
        return 0;
    else {
        if (!(st.chip_cfg.sensors))
            return -1;
        prev = st.chip_cfg.fifo_enable;
        st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors;
        if (st.chip_cfg.fifo_enable != sensors)
            /* You're not getting what you asked for. Some sensors are
             * asleep.
             */
            result = -1;
        else
            result = 0;
        if (sensors || st.chip_cfg.lp_accel_mode)
            set_int_enable(1);
        else
            set_int_enable(0);
        if (sensors) {
            if (mpu_reset_fifo()) {
                st.chip_cfg.fifo_enable = prev;
                return -1;
            }
        }
    }

    return result;
}

/**
 *  @brief      Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int mpu_get_power_state(unsigned char *power_on)
{
    if (st.chip_cfg.sensors)
        power_on[0] = 1;
    else
        power_on[0] = 0;
    return 0;
}

/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors(unsigned char sensors)
{
    unsigned char data_;
#ifdef AK89xx_SECONDARY
    unsigned char user_ctrl;
#endif

    if (sensors & INV_XYZ_GYRO)
        data_ = INV_CLK_PLL;
    else if (sensors)
        data_ = 0;
    else
        data_ = BIT_SLEEP;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data_)) {
        st.chip_cfg.sensors = 0;
        return -1;
    }
    st.chip_cfg.clk_src = data_ & ~BIT_SLEEP;

    data_ = 0;
    if (!(sensors & INV_X_GYRO))
        data_ |= BIT_STBY_XG;
    if (!(sensors & INV_Y_GYRO))
        data_ |= BIT_STBY_YG;
    if (!(sensors & INV_Z_GYRO))
        data_ |= BIT_STBY_ZG;
    if (!(sensors & INV_XYZ_ACCEL))
        data_ |= BIT_STBY_XYZA;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data_)) {
        st.chip_cfg.sensors = 0;
        return -1;
    }

    if (sensors && (sensors != INV_XYZ_ACCEL))
        /* Latched interrupts only used in LP accel mode. */
        mpu_set_int_latched(0);

#ifdef AK89xx_SECONDARY
#ifdef AK89xx_BYPASS
    if (sensors & INV_XYZ_COMPASS)
        mpu_set_bypass(1);
    else
        mpu_set_bypass(0);
#else
    if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
        return -1;
    /* Handle AKM power management. */
    if (sensors & INV_XYZ_COMPASS) {
        data_ = AKM_SINGLE_MEASUREMENT;
        user_ctrl |= BIT_AUX_IF_EN;
    } else {
        data_ = AKM_POWER_DOWN;
        user_ctrl &= ~BIT_AUX_IF_EN;
    }
    if (st.chip_cfg.dmp_on)
        user_ctrl |= BIT_DMP_EN;
    else
        user_ctrl &= ~BIT_DMP_EN;
    if (i2c_write(st.hw->addr, st.reg->s1_do, 1, &data_))
        return -1;
    /* Enable/disable I2C master mode. */
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &user_ctrl))
        return -1;
#endif
#endif

    st.chip_cfg.sensors = sensors;
    st.chip_cfg.lp_accel_mode = 0;
    delay_ms(50);
    return 0;
}

/**
 *  @brief      Read the MPU interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
int mpu_get_int_status(short *status)
{
    unsigned char tmp[2];
    if (!st.chip_cfg.sensors)
        return -1;
    if (i2c_read(st.hw->addr, st.reg->dmp_int_status, 2, tmp))
        return -1;
    status[0] = (tmp[0] << 8) | tmp[1];
    return 0;
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data_
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data_, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data_ in hardware units.
 *  @param[out] accel       Accel data_ in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
        unsigned char *sensors, unsigned char *more)
{
    /* Assumes maximum packet size is gyro (6) + accel (6). */
    unsigned char data_[MAX_PACKET_LENGTH];
    unsigned char packet_size = 0;
    unsigned short fifo_count, index = 0;

    if (st.chip_cfg.dmp_on)
        return -1;

    sensors[0] = 0;
    if (!st.chip_cfg.sensors)
        return -1;
    if (!st.chip_cfg.fifo_enable)
        return -1;

    if (st.chip_cfg.fifo_enable & INV_X_GYRO)
        packet_size += 2;
    if (st.chip_cfg.fifo_enable & INV_Y_GYRO)
        packet_size += 2;
    if (st.chip_cfg.fifo_enable & INV_Z_GYRO)
        packet_size += 2;
    if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL)
        packet_size += 6;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data_))
        return -1;
    fifo_count = (data_[0] << 8) | data_[1];
    if (fifo_count < packet_size)
        return 0;
//    log_i("FIFO count: %hd\n", fifo_count);
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, data_))
            return -1;
        if (data_[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo();
            return -2;
        }
    }
    get_ms((unsigned long*)timestamp);

    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data_))
        return -1;
    more[0] = fifo_count / packet_size - 1;
    sensors[0] = 0;

    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) {
        accel[0] = (data_[index+0] << 8) | data_[index+1];
        accel[1] = (data_[index+2] << 8) | data_[index+3];
        accel[2] = (data_[index+4] << 8) | data_[index+5];
        sensors[0] |= INV_XYZ_ACCEL;
        index += 6;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_X_GYRO) {
        gyro[0] = (data_[index+0] << 8) | data_[index+1];
        sensors[0] |= INV_X_GYRO;
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Y_GYRO) {
        gyro[1] = (data_[index+0] << 8) | data_[index+1];
        sensors[0] |= INV_Y_GYRO;
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Z_GYRO) {
        gyro[2] = (data_[index+0] << 8) | data_[index+1];
        sensors[0] |= INV_Z_GYRO;
        index += 2;
    }

    return 0;
}

/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data_    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int mpu_read_fifo_stream(unsigned short length, unsigned char *data_,
    unsigned char *more)
{
    unsigned char tmp[2];
    unsigned short fifo_count;
    if (!st.chip_cfg.dmp_on)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, tmp))
        return -1;
    fifo_count = (tmp[0] << 8) | tmp[1];
    if (fifo_count < length) {
        more[0] = 0;
        return -1;
    }
    if (fifo_count > (st.hw->max_fifo >> 1)) {
        /* FIFO is 50% full, better check overflow bit. */
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, tmp))
            return -1;
        if (tmp[0] & BIT_FIFO_OVERFLOW) {
            mpu_reset_fifo();
            return -2;
        }
    }

    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, length, data_))
        return -1;
    more[0] = fifo_count / length - 1;
    return 0;
}

/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int mpu_set_bypass(unsigned char bypass_on)
{
    unsigned char tmp;

    if (st.chip_cfg.bypass_mode == bypass_on)
        return 0;

    if (bypass_on) {
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        tmp = BIT_BYPASS_EN;
        if (st.chip_cfg.active_low_int)
            tmp |= BIT_ACTL;
        if (st.chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
            return -1;
    } else {
        /* Enable I2C master mode if compass is being used. */
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
            tmp |= BIT_AUX_IF_EN;
        else
            tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        if (st.chip_cfg.active_low_int)
            tmp = BIT_ACTL;
        else
            tmp = 0;
        if (st.chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
            return -1;
    }
    st.chip_cfg.bypass_mode = bypass_on;
    return 0;
}

/**
 *  @brief      Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int mpu_set_int_level(unsigned char active_low)
{
    st.chip_cfg.active_low_int = active_low;
    return 0;
}

/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int mpu_set_int_latched(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.latched_int == enable)
        return 0;

    if (enable)
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR;
    else
        tmp = 0;
    if (st.chip_cfg.bypass_mode)
        tmp |= BIT_BYPASS_EN;
    if (st.chip_cfg.active_low_int)
        tmp |= BIT_ACTL;
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
        return -1;
    st.chip_cfg.latched_int = enable;
    return 0;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data_        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data_)
{
    unsigned char tmp[2];

    if (!data_)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st.hw->bank_size)
        return -1;

    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->mem_r_w, length, data_))
        return -1;
    return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data_        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data_)
{
    unsigned char tmp[2];

    if (!data_)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st.hw->bank_size)
        return -1;

    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_read(st.hw->addr, st.reg->mem_r_w, length, data_))
        return -1;
    return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate)
{
    unsigned short ii = 0, jj = 0, k_avoid_compiler_optim_ = 0;
    unsigned short this_write;
    /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
    unsigned char cur[LOAD_CHUNK], firmware_chunk[LOAD_CHUNK], tmp[2];

    if (st.chip_cfg.dmp_loaded)
        /* DMP should only be loaded once. */
        return -1;

    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);
        if (eeprom_read_bytes(ii, firmware_chunk, this_write) != HAL_OK)
            return -1;
        if (mpu_write_mem(ii, this_write, firmware_chunk))
            return -1;
        if (mpu_read_mem(ii, this_write, cur))
            return -1;
        for (jj = 0; jj < this_write; jj++) {
            if (cur[jj] != firmware_chunk[jj]) {
                return -2;
            }
        }
    }

    /* Set program start address. */
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp))
        return -1;

    st.chip_cfg.dmp_loaded = 1;
    st.chip_cfg.dmp_sample_rate = sample_rate;
    return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int mpu_set_dmp_state(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.dmp_on == enable)
        return 0;

    if (enable) {
        if (!st.chip_cfg.dmp_loaded)
            return -1;
        /* Disable data_ ready interrupt. */
        set_int_enable(0);
        /* Disable bypass mode. */
        mpu_set_bypass(0);
        /* Keep constant sample rate, FIFO rate controlled by DMP. */
        mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);
        /* Remove FIFO elements. */
        tmp = 0;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 1;
        /* Enable DMP interrupt. */
        set_int_enable(1);
        mpu_reset_fifo();
    } else {
        /* Disable DMP interrupt. */
        set_int_enable(0);
        /* Restore FIFO settings. */
        tmp = st.chip_cfg.fifo_enable;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 0;
        mpu_reset_fifo();
    }
    return 0;
}

/**
 *  @brief      Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
int mpu_get_dmp_state(unsigned char *enabled)
{
    enabled[0] = st.chip_cfg.dmp_on;
    return 0;
}

#ifdef AK89xx_SECONDARY
/* This initialization is similar to the one in ak8975.c. */
static int setup_compass(void)
{
    unsigned char data_[4], akm_addr;

    mpu_set_bypass(1);

    /* Find compass. Possible addresses range from 0x0C to 0x0F. */
    for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
        int result;
        result = i2c_read(akm_addr, AKM_REG_WHOAMI, 1, data_);
        if (!result && (data_[0] == AKM_WHOAMI))
            break;
    }

    if (akm_addr > 0x0F) {
        /* TODO: Handle this case in all compass-related functions. */
        log_e("Compass not found.\n");
        return -1;
    }

    st.chip_cfg.compass_addr = akm_addr;

    data_[0] = AKM_POWER_DOWN;
    if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data_))
        return -1;
    delay_ms(1);

    data_[0] = AKM_FUSE_ROM_ACCESS;
    if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data_))
        return -1;
    delay_ms(1);

    /* Get sensitivity adjustment data_ from fuse ROM. */
    if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ASAX, 3, data_))
        return -1;
    st.chip_cfg.mag_sens_adj[0] = (long)data_[0] + 128;
    st.chip_cfg.mag_sens_adj[1] = (long)data_[1] + 128;
    st.chip_cfg.mag_sens_adj[2] = (long)data_[2] + 128;

    data_[0] = AKM_POWER_DOWN;
    if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data_))
        return -1;
    delay_ms(1);

    mpu_set_bypass(0);

    /* Set up master mode, master clock, and ES bit. */
    data_[0] = 0x40;
    if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data_))
        return -1;

    /* Slave 0 reads from AKM data_ registers. */
    data_[0] = BIT_I2C_READ | st.chip_cfg.compass_addr;
    if (i2c_write(st.hw->addr, st.reg->s0_addr, 1, data_))
        return -1;

    /* Compass reads start at this register. */
    data_[0] = AKM_REG_ST1;
    if (i2c_write(st.hw->addr, st.reg->s0_reg, 1, data_))
        return -1;

    /* Enable slave 0, 8-byte reads. */
    data_[0] = BIT_SLAVE_EN | 8;
    if (i2c_write(st.hw->addr, st.reg->s0_ctrl, 1, data_))
        return -1;

    /* Slave 1 changes AKM measurement mode. */
    data_[0] = st.chip_cfg.compass_addr;
    if (i2c_write(st.hw->addr, st.reg->s1_addr, 1, data_))
        return -1;

    /* AKM measurement mode register. */
    data_[0] = AKM_REG_CNTL;
    if (i2c_write(st.hw->addr, st.reg->s1_reg, 1, data_))
        return -1;

    /* Enable slave 1, 1-byte writes. */
    data_[0] = BIT_SLAVE_EN | 1;
    if (i2c_write(st.hw->addr, st.reg->s1_ctrl, 1, data_))
        return -1;

    /* Set slave 1 data_. */
    data_[0] = AKM_SINGLE_MEASUREMENT;
    if (i2c_write(st.hw->addr, st.reg->s1_do, 1, data_))
        return -1;

    /* Trigger slave 0 and slave 1 actions at each sample. */
    data_[0] = 0x03;
    if (i2c_write(st.hw->addr, st.reg->i2c_delay_ctrl, 1, data_))
        return -1;

#ifdef MPU9150
    /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
    data_[0] = BIT_I2C_MST_VDDIO;
    if (i2c_write(st.hw->addr, st.reg->yg_offs_tc, 1, data_))
        return -1;
#endif

    return 0;
}
#endif

/**
 *  @brief      Enters LP accel motion interrupt mode.
 *  The behaviour of this feature is very different between the MPU6050 and the
 *  MPU6500. Each chip's version of this feature is explained below.
 *
 *  \n The hardware motion threshold can be between 32mg and 8160mg in 32mg
 *  increments.
 *
 *  \n Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 5Hz, 20Hz, 40Hz
 *
 *  \n MPU6500:
 *  \n Unlike the MPU6050 version, the hardware does not "lock in" a reference
 *  sample. The hardware monitors the accel data_ and detects any large change
 *  over a short period of time.
 *
 *  \n The hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n MPU6500 Low-power accel mode supports the following frequencies:
 *  \n 0.24Hz, 0.49Hz, 0.98Hz, 1.95Hz, 3.91Hz, 7.81Hz, 15.63Hz, 31.25Hz, 62.5Hz, 125Hz, 250Hz, 500Hz
 *
 *  \n\n NOTES:
 *  \n The driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n The MPU6500 does not support a delay parameter. If this function is used
 *  for the MPU6500, the value passed to @e time will be ignored.
 *  \n To disable this mode, set @e lpa_freq to zero. The driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      Motion threshold in mg.
 *  @param[in]  time        Duration in milliseconds that the accel data_ must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    Minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
    unsigned char lpa_freq)
{

#if defined MPU6500
    unsigned char data_[3];
#endif
    if (lpa_freq) {
#if defined MPU6500
    	unsigned char thresh_hw;

        /* 1LSb = 4mg. */
        if (thresh > 1020)
            thresh_hw = 255;
        else if (thresh < 4)
            thresh_hw = 1;
        else
            thresh_hw = thresh >> 2;
#endif

        if (!time)
            /* Minimum duration must be 1ms. */
            time = 1;

#if defined MPU6500
        if (lpa_freq > 500)
            /* At this point, the chip has not been re-configured, so the
             * function can safely exit.
             */
            return -1;
#endif

        if (!st.chip_cfg.int_motion_only) {
            /* Store current settings for later. */
            if (st.chip_cfg.dmp_on) {
                mpu_set_dmp_state(0);
                st.chip_cfg.cache.dmp_on = 1;
            } else
                st.chip_cfg.cache.dmp_on = 0;
            mpu_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
            mpu_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
            mpu_get_lpf(&st.chip_cfg.cache.lpf);
            mpu_get_sample_rate(&st.chip_cfg.cache.sample_rate);
            st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
            mpu_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
        }

#if defined MPU6500
        /* Disable hardware interrupts. */
        set_int_enable(0);

        /* Enter full-power accel-only mode, no FIFO/DMP. */
        data_[0] = 0;
        data_[1] = 0;
        data_[2] = BIT_STBY_XYZG;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 3, data_))
            goto lp_int_restore;

        /* Set motion threshold. */
        data_[0] = thresh_hw;
        if (i2c_write(st.hw->addr, st.reg->motion_thr, 1, data_))
            goto lp_int_restore;

        /* Set wake frequency. */
        if (lpa_freq == 1)
            data_[0] = INV_LPA_0_98HZ;
        else if (lpa_freq == 2)
            data_[0] = INV_LPA_1_95HZ;
        else if (lpa_freq <= 5)
            data_[0] = INV_LPA_3_91HZ;
        else if (lpa_freq <= 10)
            data_[0] = INV_LPA_7_81HZ;
        else if (lpa_freq <= 20)
            data_[0] = INV_LPA_15_63HZ;
        else if (lpa_freq <= 40)
            data_[0] = INV_LPA_31_25HZ;
        else if (lpa_freq <= 70)
            data_[0] = INV_LPA_62_50HZ;
        else if (lpa_freq <= 125)
            data_[0] = INV_LPA_125HZ;
        else if (lpa_freq <= 250)
            data_[0] = INV_LPA_250HZ;
        else
            data_[0] = INV_LPA_500HZ;
        if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, data_))
            goto lp_int_restore;

        /* Enable motion interrupt (MPU6500 version). */
        data_[0] = BITS_WOM_EN;
        if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data_))
            goto lp_int_restore;
            
        /* Bypass DLPF ACCEL_FCHOICE_B=1*/
        data_[0] = BIT_ACCL_FC_B | 0x01;
        if (i2c_write(st.hw->addr, st.reg->accel_cfg2, 1, data_))
            goto lp_int_restore;

        /* Enable interrupt. */
        data_[0] = BIT_MOT_INT_EN;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data_))
            goto lp_int_restore;
        
        /* Enable cycle mode. */
        data_[0] = BIT_LPA_CYCLE;
        if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data_))
            goto lp_int_restore;
            
        st.chip_cfg.int_motion_only = 1;
        return 0;
#endif
    } else {
        /* Don't "restore" the previous state if no state has been saved. */
        int ii;
        char *cache_ptr = (char*)&st.chip_cfg.cache;
        for (ii = 0; ii < sizeof(st.chip_cfg.cache); ii++) {
            if (cache_ptr[ii] != 0)
                goto lp_int_restore;
        }
        /* If we reach this point, motion interrupt mode hasn't been used yet. */
        return -1;
    }
lp_int_restore:
    /* Set to invalid values to ensure no I2C writes are skipped. */
    st.chip_cfg.gyro_fsr = 0xFF;
    st.chip_cfg.accel_fsr = 0xFF;
    st.chip_cfg.lpf = 0xFF;
    st.chip_cfg.sample_rate = 0xFFFF;
    st.chip_cfg.sensors = 0xFF;
    st.chip_cfg.fifo_enable = 0xFF;
    st.chip_cfg.clk_src = INV_CLK_PLL;
    mpu_set_sensors(st.chip_cfg.cache.sensors_on);
    mpu_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
    mpu_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
    mpu_set_lpf(st.chip_cfg.cache.lpf);
    mpu_set_sample_rate(st.chip_cfg.cache.sample_rate);
    mpu_configure_fifo(st.chip_cfg.cache.fifo_sensors);

    if (st.chip_cfg.cache.dmp_on)
        mpu_set_dmp_state(1);

#ifdef MPU6500
    /* Disable motion interrupt (MPU6500 version). */
    data_[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data_))
        goto lp_int_restore;
#endif

    st.chip_cfg.int_motion_only = 0;
    return 0;
}

/**
 *  @}
 */


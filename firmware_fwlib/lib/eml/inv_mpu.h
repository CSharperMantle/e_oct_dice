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
 *      @file       inv_mpu.h
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */

#ifndef _INV_MPU_H_
#define _INV_MPU_H_

#include "fwlib/fw_reg_base.h"

#define __no_operation NOP

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

/* Set up APIs */
__BIT mpu_init();
__BIT mpu_init_slave(void);
__BIT mpu_set_bypass(unsigned char bypass_on);

/* Configuration APIs */
__BIT mpu_lp_accel_mode(unsigned short rate);
__BIT mpu_set_int_level(unsigned char active_low);
__BIT mpu_set_int_latched(unsigned char enable);
__BIT mpu_set_dmp_state(unsigned char enable);
__BIT mpu_set_lpf(unsigned short lpf);
__BIT mpu_set_gyro_fsr(unsigned short fsr);
__BIT mpu_set_accel_fsr(unsigned char fsr);
__BIT mpu_set_sample_rate(unsigned short rate);
__BIT mpu_configure_fifo(unsigned char sensors);
__BIT mpu_set_sensors(unsigned char sensors);

#if 0 /* DISABLED FOR UNUSED FUNCTIONS */
__BIT mpu_get_lpf(unsigned short *lpf);
__BIT mpu_get_gyro_fsr(unsigned short *fsr);
__BIT mpu_get_accel_fsr(unsigned char *fsr);
__BIT mpu_get_gyro_sens(float *sens);
__BIT mpu_get_accel_sens(unsigned short *sens);
__BIT mpu_get_sample_rate(unsigned short *rate);
__BIT mpu_get_fifo_config(unsigned char *sensors);
__BIT mpu_get_power_state(unsigned char *power_on);
#endif /* 0 */

#if 0 /* DISABLED FOR UNUSED FUNCTIONS */
__BIT mpu_read_6500_accel_bias(long *accel_bias);
#endif /* 0 */
__BIT mpu_set_gyro_bias_reg(const long * gyro_bias);
#if 0 /* DISABLED FOR UNUSED FUNCTIONS */
__BIT mpu_set_accel_bias_6500_reg(const long *accel_bias);
#endif /* 0 */

/* Data getter/setter APIs */
__BIT mpu_read_fifo(short *gyro, short *accel, unsigned char *sensors, unsigned char *more);
__BIT mpu_read_fifo_stream(unsigned short length, unsigned char *data_, unsigned char *more);
__BIT mpu_reset_fifo(void);

__BIT mpu_write_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data_);
__BIT mpu_read_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data_);
__BIT mpu_load_firmware(unsigned short length, const unsigned char __CODE *firmware,
    unsigned short start_addr, unsigned short sample_rate);

__BIT mpu_read_reg(unsigned char reg, unsigned char *data_);

#endif  /* #ifndef _INV_MPU_H_ */

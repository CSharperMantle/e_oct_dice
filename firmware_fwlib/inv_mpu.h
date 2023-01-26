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

#include "fw_reg_base.h"

#define __no_operation NOP

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

struct int_param_s {
    unsigned long pin;
    void (*cb)(volatile void*);
    void *arg;
};

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
void mpu_init(struct int_param_s *int_param);
void mpu_set_bypass(unsigned char bypass_on);

/* Configuration APIs */
void mpu_lp_accel_mode(unsigned short rate);
void mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
    unsigned char lpa_freq);
void mpu_set_int_level(unsigned char active_low);
void mpu_set_int_latched(unsigned char enable);

void mpu_set_dmp_state(unsigned char enable);
void mpu_get_dmp_state(unsigned char *enabled);

void mpu_get_lpf(unsigned short *lpf);
void mpu_set_lpf(unsigned short lpf);

void mpu_get_gyro_fsr(unsigned short *fsr);
void mpu_set_gyro_fsr(unsigned short fsr);

void mpu_get_accel_fsr(unsigned char *fsr);
void mpu_set_accel_fsr(unsigned char fsr);

void mpu_get_gyro_sens(float *sens);
void mpu_get_accel_sens(unsigned short *sens);

void mpu_get_sample_rate(unsigned short *rate);
void mpu_set_sample_rate(unsigned short rate);

void mpu_get_fifo_config(unsigned char *sensors);
void mpu_configure_fifo(unsigned char sensors);

void mpu_get_power_state(unsigned char *power_on);
void mpu_set_sensors(unsigned char sensors);

/* Data getter/setter APIs */
void mpu_get_gyro_reg(short *data_);
void mpu_get_accel_reg(short *data_);
void mpu_get_temperature(long *data_);

void mpu_get_int_status(short *status);
void mpu_read_fifo(short *gyro, short *accel, unsigned char *sensors, unsigned char *more);
void mpu_read_fifo_stream(unsigned short length, unsigned char *data_, unsigned char *more);
void mpu_reset_fifo(void);

void mpu_reg_dump(void);
void mpu_read_reg(unsigned char reg, unsigned char *data_);

#endif  /* #ifndef _INV_MPU_H_ */


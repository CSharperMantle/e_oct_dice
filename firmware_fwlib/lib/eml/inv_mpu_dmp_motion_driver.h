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
 *      @file       inv_mpu_dmp_motion_driver.h
 *      @brief      DMP image and interface functions.
 *      @details    All functions are preceded by the dmp_ prefix to
 *                  differentiate among MPL and general driver function calls.
 */
#ifndef _INV_MPU_DMP_MOTION_DRIVER_H_
#define _INV_MPU_DMP_MOTION_DRIVER_H_

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)

#define INV_WXYZ_QUAT       (0x100)

/* Set up functions. */
__BIT dmp_load_motion_driver_firmware(void);
__BIT dmp_set_fifo_rate(unsigned short rate);
__BIT dmp_get_fifo_rate(unsigned short *rate);
__BIT dmp_enable_feature(unsigned short mask);
__BIT dmp_get_enabled_features(unsigned short *mask);
__BIT dmp_set_interrupt_mode(unsigned char mode);
__BIT dmp_set_gyro_bias(long *bias);
__BIT dmp_set_accel_bias(long *bias);

/* LP quaternion functions. */
__BIT dmp_enable_lp_quat(__BIT enable);
__BIT dmp_enable_6x_lp_quat(__BIT enable);

/* DMP gyro calibration functions. */
__BIT dmp_enable_gyro_cal(__BIT enable);

/* Read function. This function should be called whenever the MPU interrupt is
 * detected.
 */
__BIT dmp_read_fifo(short *gyro, short *accel, long *quat, short *sensors, unsigned char *more);

#endif  /* #ifndef _INV_MPU_DMP_MOTION_DRIVER_H_ */


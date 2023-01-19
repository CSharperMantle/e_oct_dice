#include <stdio.h>
#include <math.h>
#include "fw_hal.h"
#include "inv_mpu.h"
#include "MahonyAHRS.h"

#define EPSILON         0.0001f
#define PI_2            1.57079632679489661923f

#define PIN_LED_PYR P00
#define PIN_LED_HYD P01
#define PIN_LED_ANE P02
#define PIN_LED_ELE P03
#define PIN_LED_DEN P04
#define PIN_LED_CYR P05
#define PIN_LED_GEO P06
#define PIN_LED_WLD P07

#define MPU_CALIB_GYRO_X_GOAL 0
#define MPU_CALIB_GYRO_Y_GOAL 0
#define MPU_CALIB_GYRO_Z_GOAL 0
#define MPU_CALIB_ACCEL_X_GOAL 0
#define MPU_CALIB_ACCEL_Y_GOAL 0
#define MPU_CALIB_ACCEL_Z_GOAL 16384
#define MPU_BIAS_SAMPLES 200
#define MPU_GYRO_FSR 2000
#define MPU_REFRESH_RATE_HZ 100
#define MPU_F_GYRO_SENS 16.375f
#define MPU_F_ACCEL_SENS 16384.f

__DATA int i;

__IDATA short gyro_s[3], accel_s[3];
__IDATA unsigned char more, sensors_mpu;
__IDATA float gyro_f[3], accel_f[3], euler[3];

/* MPU Calibration */
__IDATA long gyro_bias_l[3] = {0}, accel_bias_l[3] = {0};

/* = = = QUATERNION AND VECTOR HELPER FUNCTIONS = = = */

/*
static float vec3_dot(float *vec3_x, float *vec3_y) small {
    return (vec3_x[0] * vec3_y[0]
          + vec3_x[1] * vec3_y[1]
          + vec3_x[2] * vec3_y[2]
          + vec3_x[3] * vec3_y[3]);
}

static void vec3_cross(float *vec3_x, float *vec3_y, float *vec3_out) small {
    vec3_out[0] = vec3_x[1] * vec3_y[2] - vec3_y[1] * vec3_x[2];
    vec3_out[1] = vec3_x[2] * vec3_y[0] - vec3_y[2] * vec3_x[0];
    vec3_out[2] = vec3_x[0] * vec3_y[1] - vec3_y[0] * vec3_x[1];
}

static void vec3_mul(float *vec3_x, float s, float *vec3_out) small {
    vec3_out[0] = vec3_x[0] * s;
    vec3_out[1] = vec3_x[1] * s;
    vec3_out[2] = vec3_x[2] * s;
}

static void vec3_add(float *vec3_x, float *vec3_y, float *vec3_out) small {
    vec3_out[0] = vec3_x[0] + vec3_y[0];
    vec3_out[1] = vec3_x[1] + vec3_y[1];
    vec3_out[2] = vec3_x[2] + vec3_y[2];
}

static void rotate(float *vec3_v, float *vec4_q, float *vec3_out) small {
    float s_tmp_1, s_tmp_2;
    float *vec3_u;
    float vec3_tmp_1[3], vec3_tmp_2[3];
    
    vec3_u = &vec4_q[1];

    s_tmp_1 = vec3_dot(vec3_u, vec3_v);
    s_tmp_1 *= 2.f;
    vec3_mul(vec3_u, s_tmp_1, vec3_tmp_1);

    s_tmp_1 = vec4_q[0];

    s_tmp_2 = vec3_dot(vec3_u, vec3_u);
    s_tmp_2 = -(s_tmp_2);
    s_tmp_2 += s_tmp_1 * s_tmp_1;
    vec3_mul(vec3_v, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_tmp_1);

    s_tmp_2 = 2.f * s_tmp_1;
    vec3_cross(vec3_u, vec3_v, vec3_tmp_2);
    vec3_mul(vec3_tmp_2, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_out);
}
*/

/* = = = END OF QUATERNION AND VECTOR HELPER FUNCTIONS = = = */

void main(void) small {
    // LEDs
    GPIO_P0_SetMode(GPIO_Pin_All, GPIO_Mode_Output_PP);
    PIN_LED_PYR = 0;
    PIN_LED_HYD = 0;
    PIN_LED_ANE = 0;
    PIN_LED_ELE = 0;
    PIN_LED_DEN = 0;
    PIN_LED_CYR = 0;
    PIN_LED_GEO = 0;
    PIN_LED_WLD = 0;
    // Serial
    GPIO_P3_SetMode(GPIO_Pin_6, GPIO_Mode_Input_HIP);
    GPIO_P3_SetMode(GPIO_Pin_7, GPIO_Mode_Output_PP);
    // I2C
    GPIO_P1_SetMode(GPIO_Pin_4, GPIO_Mode_InOut_OD);  // SDA
    GPIO_P1_SetMode(GPIO_Pin_5, GPIO_Mode_InOut_OD);  // SCL
    GPIO_SetPullUp(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, HAL_State_ON);
    GPIO_SetSwitchSpeed(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, GPIO_SwitchSpeed_High);
    GPIO_SetDriveCapability(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, GPIO_DriveCapability_High);
    GPIO_SetSchmittTrigger(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, GPIO_SchmittTrigger_ON);
    GPIO_SetDigitalInput(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, HAL_State_ON);

    UART1_SwitchPort(UART1_AlterPort_P36_P37);
    UART1_24M_115200_Init();

    I2C_SetWorkMode(I2C_WorkMode_Master);
    I2C_SetClockPrescaler(13U);
    I2C_SetPort(I2C_AlterPort_P15_P14);
    I2C_SetEnabled(HAL_State_ON);

    mpu_init(NULL);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_gyro_fsr(MPU_GYRO_FSR);
    mpu_set_accel_fsr(2);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(MPU_REFRESH_RATE_HZ);
    printf("MPU OK\r\n");

    SYS_Delay(250);

    // IMU warmup
    for (i = 0; i < MPU_BIAS_SAMPLES; i++) {
        do {
            mpu_read_fifo(gyro_s, accel_s, NULL, &sensors_mpu, &more);
        } while (more);
        gyro_bias_l[0] += MPU_CALIB_GYRO_X_GOAL - (long)gyro_s[0];
        gyro_bias_l[1] += MPU_CALIB_GYRO_Y_GOAL - (long)gyro_s[1];
        gyro_bias_l[2] += MPU_CALIB_GYRO_Z_GOAL - (long)gyro_s[2];
        accel_bias_l[0] += MPU_CALIB_ACCEL_X_GOAL - (long)accel_s[0];
        accel_bias_l[1] += MPU_CALIB_ACCEL_Y_GOAL - (long)accel_s[1];
        accel_bias_l[2] += MPU_CALIB_ACCEL_Z_GOAL - (long)accel_s[2];
        SYS_Delay(2);
    }
    gyro_bias_l[0] /= MPU_BIAS_SAMPLES;
    gyro_bias_l[1] /= MPU_BIAS_SAMPLES;
    gyro_bias_l[2] /= MPU_BIAS_SAMPLES;
    accel_bias_l[0] /= MPU_BIAS_SAMPLES;
    accel_bias_l[1] /= MPU_BIAS_SAMPLES;
    accel_bias_l[2] /= MPU_BIAS_SAMPLES;

    // Main loop
    while (1) {
        do {
            mpu_read_fifo(gyro_s, accel_s, NULL, &sensors_mpu, &more);

            gyro_s[0] += (short)gyro_bias_l[0];
            gyro_s[1] += (short)gyro_bias_l[1];
            gyro_s[2] += (short)gyro_bias_l[2];
            accel_s[0] += (short)accel_bias_l[0];
            accel_s[1] += (short)accel_bias_l[1];
            accel_s[2] += (short)accel_bias_l[2];

            gyro_f[0] = ((float)gyro_s[0]) / MPU_F_GYRO_SENS;
            gyro_f[1] = ((float)gyro_s[1]) / MPU_F_GYRO_SENS;
            gyro_f[2] = ((float)gyro_s[2]) / MPU_F_GYRO_SENS;

            accel_f[0] = ((float)accel_s[0]) / MPU_F_ACCEL_SENS;
            accel_f[1] = ((float)accel_s[1]) / MPU_F_ACCEL_SENS;
            accel_f[2] = ((float)accel_s[2]) / MPU_F_ACCEL_SENS;

            MahonyAHRSUpdateIMU(gyro_f[0], gyro_f[1], gyro_f[2], accel_f[0], accel_f[1], accel_f[2]);
        } while (more);

        // printf("%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\r\n", 
        //     gyro_f[0], gyro_f[1], gyro_f[2],
        //     accel_f[0], accel_f[1], accel_f[2]);

        // quat_to_euler(&euler[0], &euler[1], &euler[2]);
        // euler[0] = RAD_TO_DEG(euler[0]);
        // euler[1] = RAD_TO_DEG(euler[1]);
        // euler[2] = RAD_TO_DEG(euler[2]);

        printf("%.6f\t%.6f\t%.6f\t%.6f\r\n", q0, q1, q2, q3);

        // printf("%.6f\t%.6f\t%.6f\r\n", euler[0], euler[1], euler[2]);

        SYS_Delay(5);
    }
}

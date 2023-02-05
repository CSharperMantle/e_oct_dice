#include <stdio.h>
#include <math.h>
#include "fwlib/fw_hal.h"
#include "eml/inv_mpu.h"
#include "arhs/MahonyAHRS.h"

#define HALF_SQRT2 (0.7071067811865f)

#define PIN_LED_PYR P00
#define PIN_LED_HYD P01
#define PIN_LED_ANE P02
#define PIN_LED_ELE P03
#define PIN_LED_DEN P04
#define PIN_LED_CYR P05
#define PIN_LED_GEO P06
#define PIN_LED_WLD P07

#define MASK_LED_PYR B00000001
#define MASK_LED_HYD B00000010
#define MASK_LED_ANE B00000100
#define MASK_LED_ELE B00001000
#define MASK_LED_DEN B00010000
#define MASK_LED_CYR B00100000
#define MASK_LED_GEO B01000000
#define MASK_LED_WLD B10000000

#define IS_IN_RANGE_EPS(x, t, eps) (((x) > ((t) - (eps))) && ((x) < ((t) + (eps))))

#define MPU_CALIB_GYRO_X_GOAL 0
#define MPU_CALIB_GYRO_Y_GOAL 0
#define MPU_CALIB_GYRO_Z_GOAL 0
#define MPU_CALIB_ACCEL_X_GOAL 0
#define MPU_CALIB_ACCEL_Y_GOAL 0
#define MPU_CALIB_ACCEL_Z_GOAL 16384
#define MPU_READ_SAMPLES 8
#define MPU_CALIB_SAMPLES 64
#define MPU_CALIB_ROUNDS 8
#define MPU_GYRO_FSR 2000
#define MPU_REFRESH_RATE_HZ 100
#define MPU_F_GYRO_SENS 16.375f
#define MPU_F_ACCEL_SENS 16384.0f

#define STEADY_THRESHOLD 3

/* Array of initial face orientation vectors */
__CODE const float ARR_VEC3_FACES_ORIENT[8][3] = {
    {HALF_SQRT2, 0.0f, HALF_SQRT2},    /* PYR, X+ Z+ */
    {0.0f, HALF_SQRT2, HALF_SQRT2},    /* HYD, Y+ Z+ */
    {-HALF_SQRT2, 0.0f, HALF_SQRT2},   /* ANE, X- Z+ */
    {0.0f, -HALF_SQRT2, HALF_SQRT2},   /* ELE, Y- Z+ */
    {HALF_SQRT2, 0.0f, -HALF_SQRT2},   /* DEN, X+ Z- */
    {0.0f, HALF_SQRT2, -HALF_SQRT2},   /* CYR, Y+ Z- */
    {-HALF_SQRT2, 0.0f, -HALF_SQRT2},  /* GEO, X- Z- */
    {0.0f, -HALF_SQRT2, -HALF_SQRT2},  /* WLD, Y- Z- */
};

/* Accumulator */
__DATA int i = 0, j = 0;
/* Raw gyro/accel readings */
__DATA short gyro_s[3] = {0}, accel_s[3] = {0};
/* MPU FIFO status */
__DATA unsigned char more = 0, sensors_mpu = 0;
/* Computed orientation */
__IDATA float gyro_f[3] = {0}, accel_f[3] = {0}, q_f[4] = {0};
/* Summed gyro/accel readings, to be averaged */
__IDATA long gyro_sum_l[3] = {0}, accel_sum_l[3] = {0};
/* Flag indicating if the sensor is stationery or steady */
__BIT is_steady = 0;
/* Delta of bias of gyro/accel, used in calibration */
__IDATA long gyro_calib_delta_l[3] = {0}, accel_calib_delta_l[3] = {0};
/* Bias for gyro/accel, to be added to final readings */
__IDATA short gyro_bias_s[3] = {0}, accel_bias_s[3] = {0};

#define DEFINE_ACTIVATION_SHRINK(name, lambda)  \
static float name(float x) small {  \
    return (x > -(lambda) && x < (lambda)) ? 0.0f : x;  \
}

DEFINE_ACTIVATION_SHRINK(gyro_shrink, 0.375f)

static void vec3_dot(const float *vec3_x, const float *vec3_y, float *s_out) small {
    *s_out = (vec3_x[0] * vec3_y[0]
            + vec3_x[1] * vec3_y[1]
            + vec3_x[2] * vec3_y[2]);
}

static void vec3_cross(const float *vec3_x, const float *vec3_y, float *vec3_out) small {
    vec3_out[0] = vec3_x[1] * vec3_y[2] - vec3_y[1] * vec3_x[2];
    vec3_out[1] = vec3_x[2] * vec3_y[0] - vec3_y[2] * vec3_x[0];
    vec3_out[2] = vec3_x[0] * vec3_y[1] - vec3_y[0] * vec3_x[1];
}

static void vec3_mul(const float *vec3_x, float s, float *vec3_out) small {
    vec3_out[0] = vec3_x[0] * s;
    vec3_out[1] = vec3_x[1] * s;
    vec3_out[2] = vec3_x[2] * s;
}

static void vec3_add(const float *vec3_x, const float *vec3_y, float *vec3_out) small {
    vec3_out[0] = vec3_x[0] + vec3_y[0];
    vec3_out[1] = vec3_x[1] + vec3_y[1];
    vec3_out[2] = vec3_x[2] + vec3_y[2];
}

static void rotate(const float *vec3_v, const float *vec4_q, float *vec3_out) compact {
    __PDATA float s_tmp_1 = 0.0f, s_tmp_2 = 0.0f;
    __PDATA float *vec3_u = NULL;
    __PDATA float vec3_tmp_1[3] = 0.0f, vec3_tmp_2[3] = 0.0f;

    vec3_out[0] = vec3_out[1] = vec3_out[2] = 0.0f;
    
    vec3_u = &vec4_q[1];

    vec3_dot(vec3_u, vec3_v, &s_tmp_1);
    s_tmp_1 *= 2.0f;
    vec3_mul(vec3_u, s_tmp_1, vec3_tmp_1);

    s_tmp_1 = vec4_q[0];

    vec3_dot(vec3_u, vec3_u, &s_tmp_2);
    s_tmp_2 = -(s_tmp_2);
    s_tmp_2 += s_tmp_1 * s_tmp_1;
    vec3_mul(vec3_v, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_tmp_1);

    s_tmp_2 = 2.0f * s_tmp_1;
    vec3_cross(vec3_u, vec3_v, vec3_tmp_2);
    vec3_mul(vec3_tmp_2, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_out);
}

/*
 * Refresh MPU data by batch-reading from FIFO and averaging.
 * This function also adds bias to the readings.
 *
 * Affects `gyro_sum_l`, `accel_sum_l`.
 * Results stored in `gyro_s`, `accel_s`.
 */
static void refresh_mpu(void) small {
    __DATA int count = 0;

    gyro_sum_l[0] = gyro_sum_l[1] = gyro_sum_l[2] = 0;
    accel_sum_l[0] = accel_sum_l[1] = accel_sum_l[2] = 0;

    do {
        mpu_read_fifo(gyro_s, accel_s, &sensors_mpu, &more);
        gyro_sum_l[0] += (long)gyro_s[0];
        gyro_sum_l[1] += (long)gyro_s[1];
        gyro_sum_l[2] += (long)gyro_s[2];
        accel_sum_l[0] += (long)accel_s[0];
        accel_sum_l[1] += (long)accel_s[1];
        accel_sum_l[2] += (long)accel_s[2];
        count++;
    } while (more && (count <= MPU_READ_SAMPLES));

    gyro_s[0] = (short)(gyro_sum_l[0] / count) + gyro_bias_s[0];
    gyro_s[1] = (short)(gyro_sum_l[1] / count) + gyro_bias_s[1];
    gyro_s[2] = (short)(gyro_sum_l[2] / count) + gyro_bias_s[2];
    accel_s[0] = (short)(accel_sum_l[0] / count) + accel_bias_s[0];
    accel_s[1] = (short)(accel_sum_l[1] / count) + accel_bias_s[1];
    accel_s[2] = (short)(accel_sum_l[2] / count) + accel_bias_s[2];

    is_steady = (IS_IN_RANGE_EPS(gyro_s[0], 0, STEADY_THRESHOLD)
              && IS_IN_RANGE_EPS(gyro_s[1], 0, STEADY_THRESHOLD)
              && IS_IN_RANGE_EPS(gyro_s[2], 0, STEADY_THRESHOLD));
}

static void set_led(unsigned char leds) small {
    PIN_LED_PYR = leds & MASK_LED_PYR;
    PIN_LED_HYD = leds & MASK_LED_HYD;
    PIN_LED_ANE = leds & MASK_LED_ANE;
    PIN_LED_ELE = leds & MASK_LED_ELE;
    PIN_LED_DEN = leds & MASK_LED_DEN;
    PIN_LED_CYR = leds & MASK_LED_CYR;
    PIN_LED_GEO = leds & MASK_LED_GEO;
    PIN_LED_WLD = leds & MASK_LED_WLD;
}

/*
 * Determine which face up from computed quaternion.
 *
 * Results is returned.
 */
static unsigned char get_led_mask_by_q(float *vec4_q) small {
    __IDATA float vec3_out[3];
    __IDATA unsigned char final_mask = 0;

    for (i = 0; i < 1; i++) {
        rotate(ARR_VEC3_FACES_ORIENT[i], vec4_q, vec3_out);
        // printf("#%d,%f,%f,%f\r\n", i, vec3_out[0], vec3_out[1], vec3_out[2]);
        if (IS_IN_RANGE_EPS(vec3_out[0], 0.0f, 0.2f)
         && IS_IN_RANGE_EPS(vec3_out[0], 0.0f, 0.2f)
         && IS_IN_RANGE_EPS(vec3_out[0], 1.0f, 0.2f)) {
            final_mask |= 0x01u << i;
        }
    }

    return final_mask;
}

void main(void) small {
    EXTI_Global_SetIntState(HAL_State_ON);

    /* LEDs */
    GPIO_P0_SetMode(GPIO_Pin_All, GPIO_Mode_Output_PP);
    set_led(0);
    /* Serial */
    GPIO_P3_SetMode(GPIO_Pin_6, GPIO_Mode_Input_HIP);
    GPIO_P3_SetMode(GPIO_Pin_7, GPIO_Mode_Output_PP);
    /* I2C */
    GPIO_P1_SetMode(GPIO_Pin_4, GPIO_Mode_InOut_OD);  /* SDA */
    GPIO_P1_SetMode(GPIO_Pin_5, GPIO_Mode_InOut_OD);  /* SCL */
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

    SYS_Delay(100);
    
    /* IMU calibration */

    set_led(MASK_LED_WLD);
    for (i = 0; i < MPU_CALIB_ROUNDS; i++) {
        for (j = 0; j < MPU_CALIB_SAMPLES; j++) {
            refresh_mpu();
            gyro_calib_delta_l[0] += MPU_CALIB_GYRO_X_GOAL - (long)gyro_s[0];
            gyro_calib_delta_l[1] += MPU_CALIB_GYRO_Y_GOAL - (long)gyro_s[1];
            gyro_calib_delta_l[2] += MPU_CALIB_GYRO_Z_GOAL - (long)gyro_s[2];
            accel_calib_delta_l[0] += MPU_CALIB_ACCEL_X_GOAL - (long)accel_s[0];
            accel_calib_delta_l[1] += MPU_CALIB_ACCEL_Y_GOAL - (long)accel_s[1];
            accel_calib_delta_l[2] += MPU_CALIB_ACCEL_Z_GOAL - (long)accel_s[2];
            SYS_Delay(5);
        }
        gyro_calib_delta_l[0] /= MPU_CALIB_SAMPLES;
        gyro_calib_delta_l[1] /= MPU_CALIB_SAMPLES;
        gyro_calib_delta_l[2] /= MPU_CALIB_SAMPLES;
        accel_calib_delta_l[0] /= MPU_CALIB_SAMPLES;
        accel_calib_delta_l[1] /= MPU_CALIB_SAMPLES;
        accel_calib_delta_l[2] /= MPU_CALIB_SAMPLES;

        gyro_bias_s[0] += (short)gyro_calib_delta_l[0];
        gyro_bias_s[1] += (short)gyro_calib_delta_l[1];
        gyro_bias_s[2] += (short)gyro_calib_delta_l[2];
        accel_bias_s[0] += (short)accel_calib_delta_l[0];
        accel_bias_s[1] += (short)accel_calib_delta_l[1];
        accel_bias_s[2] += (short)accel_calib_delta_l[2];

        SYS_Delay(10);
    }
    set_led(0);

    /* Main loop */
    while (1) {
        refresh_mpu();

        gyro_f[0] = gyro_shrink(((float)gyro_s[0]) / MPU_F_GYRO_SENS);
        gyro_f[1] = gyro_shrink(((float)gyro_s[1]) / MPU_F_GYRO_SENS);
        gyro_f[2] = gyro_shrink(((float)gyro_s[2]) / MPU_F_GYRO_SENS);

        accel_f[0] = ((float)accel_s[0]) / MPU_F_ACCEL_SENS;
        accel_f[1] = ((float)accel_s[1]) / MPU_F_ACCEL_SENS;
        accel_f[2] = ((float)accel_s[2]) / MPU_F_ACCEL_SENS;

        MahonyAHRSUpdateIMU(gyro_f[0], gyro_f[1], gyro_f[2], accel_f[0], accel_f[1], accel_f[2], 0.005f);
        q_f[0] = q0;
        q_f[1] = q1;
        q_f[2] = q2;
        q_f[3] = q3;

        printf("%f,%f,%f\r\n", accel_f[0], accel_f[1], accel_f[2]);

        set_led(get_led_mask_by_q(q_f));

        SYS_Delay(10);
    }
}

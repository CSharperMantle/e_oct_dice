#include <stdio.h>
#include <math.h>
#include "fwlib/fw_hal.h"
#include "eml/inv_mpu.h"
#include "eml/inv_mpu_dmp_motion_driver.h"
#include "arhs/MahonyAHRS.h"

#define HALF_SQRT2 (0.7071067811865f)
#define Q30 (1073741824.0f)

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
#define MPU_CALIB_SAMPLES 256
#define MPU_REFRESH_RATE_HZ 200

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
static __DATA int i = 0;
/* Raw gyro readings */
static __DATA short gyro_s[3] = {0}, accel_s[3] = {0};
/* MPU FIFO status */
static __DATA unsigned char more = 0;
static __DATA short sensors_mpu = 0;
/* Computed orientation */
static __DATA long quat_l[4] = {0};
static __DATA float quat_f[4] = {0};
/* Bias for gyro, to be added to final readings */
static __IDATA long gyro_bias_l[3] = {0};

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

static void rotate(const float *vec3_v, const float *vec4_q, float *vec3_out) small {
    __IDATA float s_tmp_1 = 0.0f, s_tmp_2 = 0.0f;
    __IDATA float *vec3_u = NULL;
    __IDATA float vec3_tmp_1[3] = {0.0f, 0.0f, 0.0f}, vec3_tmp_2[3] = {0.0f, 0.0f, 0.0f};

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

static void refresh_mpu(void) small {
    do {
        mpu_read_fifo(gyro_s, NULL, &sensors_mpu, &more);
    } while (more);
}

static void refresh_dmp(void) small {
    do {
        dmp_read_fifo(gyro_s, NULL, quat_l, &sensors_mpu, &more);
    } while (more);
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
    __DATA unsigned char final_mask = 0;

    for (i = 0; i < 8; i++) {
        rotate(ARR_VEC3_FACES_ORIENT[i], vec4_q, vec3_out);
        if (IS_IN_RANGE_EPS(vec3_out[0], 0.0f, 0.25f)
           && IS_IN_RANGE_EPS(vec3_out[1], 0.0f, 0.25f)
           && IS_IN_RANGE_EPS(vec3_out[2], 1.0f, 0.25f)) {
            final_mask |= 0x01u << i;
            printf("#%d,%f,%f,%f\r\n", i, vec3_out[0], vec3_out[1], vec3_out[2]);
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
    mpu_set_gyro_fsr(1000);
    mpu_set_accel_fsr(8);
    mpu_set_lpf(5);
    mpu_configure_fifo(INV_XYZ_GYRO);
    mpu_set_sample_rate(MPU_REFRESH_RATE_HZ);

    SYS_Delay(1000);
    i = MPU_CALIB_SAMPLES;
    do {
        refresh_mpu();
        gyro_bias_l[0] += (long)(gyro_s[0] - MPU_CALIB_GYRO_X_GOAL);
        gyro_bias_l[1] += (long)(gyro_s[1] - MPU_CALIB_GYRO_Y_GOAL);
        gyro_bias_l[2] += (long)(gyro_s[2] - MPU_CALIB_GYRO_Z_GOAL);
        SYS_Delay(10);
    } while (--i);
    gyro_bias_l[0] /= MPU_CALIB_SAMPLES;
    gyro_bias_l[1] /= MPU_CALIB_SAMPLES;
    gyro_bias_l[2] /= MPU_CALIB_SAMPLES;
    printf("MPU GYRO CALIB BIAS:\t%d\t%d\t%d\r\n", (int)gyro_bias_l[0], (int)gyro_bias_l[1], (int)gyro_bias_l[2]);
    SYS_Delay(1000);

    dmp_load_motion_driver_firmware();
    dmp_set_fifo_rate(MPU_REFRESH_RATE_HZ);
    mpu_set_dmp_state(1);
    mpu_set_gyro_bias_reg(gyro_bias_l);
    dmp_set_gyro_bias(gyro_bias_l);
    dmp_enable_feature(DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_6X_LP_QUAT);
    SYS_Delay(100);

    do {
        refresh_dmp();
        quat_f[0] = (float)quat_l[0] / Q30;
        quat_f[1] = (float)quat_l[1] / Q30;
        quat_f[2] = (float)quat_l[2] / Q30;
        quat_f[3] = (float)quat_l[3] / Q30;
        // printf("%f\t%f\t%f\t%f\r\n", quat_f[0], quat_f[1], quat_f[2], quat_f[3]);
        set_led(get_led_mask_by_q(quat_f));
        SYS_Delay(10);
    } while (1);
}

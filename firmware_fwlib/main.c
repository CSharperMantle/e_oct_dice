#include <stdio.h>
#include "fwlib/fw_hal.h"
#include "eml/inv_mpu.h"
#include "eml/inv_mpu_dmp_motion_driver.h"

#define HALF_SQRT2 (0.7071067811865f)
#define Q30 (1073741824.0f)

#if defined(E_OCT_DICE_DEV_STC8H8K64U)

#define PIN_LED_PYR P00
#define PIN_LED_HYD P01
#define PIN_LED_ANE P02
#define PIN_LED_ELE P03
#define PIN_LED_DEN P04
#define PIN_LED_CYR P05
#define PIN_LED_GEO P06
#define PIN_LED_WLD P07

static void init_periph(void) small {
    /* LEDs */
    GPIO_P0_SetMode(GPIO_Pin_All, GPIO_Mode_Output_PP);
    /* Serial */
    GPIO_P3_SetMode(GPIO_Pin_6, GPIO_Mode_Input_HIP);
    GPIO_P3_SetMode(GPIO_Pin_7, GPIO_Mode_Output_PP);
    /* I2C */
    GPIO_P1_SetMode(GPIO_Pin_4, GPIO_Mode_InOut_OD);  /* SDA */
    GPIO_P1_SetMode(GPIO_Pin_5, GPIO_Mode_InOut_OD);  /* SCL */
    GPIO_SetPullUp(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, HAL_State_ON);

    UART1_SwitchPort(UART1_AlterPort_P36_P37);
    UART1_24M_115200_Init();

    I2C_SetWorkMode(I2C_WorkMode_Master);
    I2C_SetClockPrescaler(13U);
    I2C_SetPort(I2C_AlterPort_P15_P14);
    I2C_SetEnabled(HAL_State_ON);
}

#elif defined(E_OCT_DICE_REL_STC8H1K28)

#define PIN_LED_PYR P24
#define PIN_LED_HYD P25
#define PIN_LED_ANE P26
#define PIN_LED_ELE P27
#define PIN_LED_DEN P20
#define PIN_LED_CYR P21
#define PIN_LED_GEO P22
#define PIN_LED_WLD P23

static void init_periph(void) small {
    /* LEDs */
    GPIO_P2_SetMode(GPIO_Pin_All, GPIO_Mode_Output_PP);
    /* Serial */
    GPIO_P3_SetMode(GPIO_Pin_0, GPIO_Mode_Input_HIP);
    GPIO_P3_SetMode(GPIO_Pin_1, GPIO_Mode_Output_PP);
    /* I2C */
    GPIO_P1_SetMode(GPIO_Pin_4, GPIO_Mode_InOut_OD);  /* SDA */
    GPIO_P1_SetMode(GPIO_Pin_5, GPIO_Mode_InOut_OD);  /* SCL */
    GPIO_SetPullUp(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, HAL_State_ON);

    UART1_SwitchPort(UART1_AlterPort_P30_P31);
    UART1_24M_115200_Init();

    I2C_SetWorkMode(I2C_WorkMode_Master);
    I2C_SetClockPrescaler(13U);
    I2C_SetPort(I2C_AlterPort_P15_P14);
    I2C_SetEnabled(HAL_State_ON);
}

#elif defined(E_OCT_DICE_REL_STC8G1K17)

#define PIN_LED_PYR P10
#define PIN_LED_HYD P13
#define PIN_LED_ANE P12
#define PIN_LED_ELE P11
#define PIN_LED_DEN P33
#define PIN_LED_CYR P34
#define PIN_LED_GEO P35
#define PIN_LED_WLD P36

static void init_periph(void) small {
    /* LEDs */
    GPIO_P1_SetMode(GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_Output_PP);
    GPIO_P3_SetMode(GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6, GPIO_Mode_Output_PP);
    /* Serial */
    GPIO_P3_SetMode(GPIO_Pin_0, GPIO_Mode_Input_HIP);
    GPIO_P3_SetMode(GPIO_Pin_1, GPIO_Mode_Output_PP);
    /* I2C */
    GPIO_P1_SetMode(GPIO_Pin_4, GPIO_Mode_InOut_OD);  /* SDA */
    GPIO_P1_SetMode(GPIO_Pin_5, GPIO_Mode_InOut_OD);  /* SCL */
    GPIO_SetPullUp(GPIO_Port_1, GPIO_Pin_4 | GPIO_Pin_5, HAL_State_ON);

    UART1_SwitchPort(UART1_AlterPort_P30_P31);
    UART1_24M_115200_Init();

    I2C_SetWorkMode(I2C_WorkMode_Master);
    I2C_SetClockPrescaler(13U);
    I2C_SetPort(I2C_AlterPort_P15_P14);
    I2C_SetEnabled(HAL_State_ON);
}

#else
#error Build type misconfigured.
#endif

#define MASK_LED_PYR B00000001
#define MASK_LED_HYD B00000010
#define MASK_LED_ANE B00000100
#define MASK_LED_ELE B00001000
#define MASK_LED_DEN B00010000
#define MASK_LED_CYR B00100000
#define MASK_LED_GEO B01000000
#define MASK_LED_WLD B10000000

#define IS_IN_RANGE_EPS(x, t, eps) (((x) > ((t) - (eps))) && ((x) < ((t) + (eps))))

#define MPU_SIGNATURE_WHOAMI 0x70
#define MPU_REG_WHOAMI 0x75
#define MPU_CALIB_GYRO_X_GOAL 0
#define MPU_CALIB_GYRO_Y_GOAL 0
#define MPU_CALIB_GYRO_Z_GOAL 0
#define MPU_CALIB_SAMPLES 128
#define MPU_REFRESH_RATE_HZ 200

#define MSG_FMT_INFO_MPU_INIT_BEGIN "I\tIB\tMPU\r\n"
#define MSG_FMT_INFO_MPU_INIT_END "I\tIE\tMPU\r\n"
#define MSG_FMT_INFO_MPU_BIAS_CALIB_DONE "I\tD\tMPU\tBIAS\t%d\t%d\t%d\r\n"
#define MSG_FMT_INFO_FACE_DETECTED "I\tF\t%d\t%f\t%f\t%f\r\n"
#define MSG_FMT_ERR_MPU_WHOAMI_MISMATCH "E\tH\tMPU\tWHOAMI\t%d\t%d\r\n"

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

/* Raw readings */
static __DATA short gyro_s[3] = {0};
/* Computed orientation */
static __DATA float quat_f[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/* Bias for gyro, to be added to final readings */
static __IDATA long gyro_bias_l[3] = {0};

static float vec3_dot(const float *vec3_x, const float *vec3_y) small {
    return (vec3_x[0] * vec3_y[0]
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
    float s_tmp_1 = 0.0f, s_tmp_2 = 0.0f;
    float *vec3_u = NULL;
    float vec3_tmp_1[3] = {0.0f, 0.0f, 0.0f}, vec3_tmp_2[3] = {0.0f, 0.0f, 0.0f};

    vec3_out[0] = vec3_out[1] = vec3_out[2] = 0.0f;
    
    vec3_u = &vec4_q[1];

    s_tmp_1 = vec3_dot(vec3_u, vec3_v);
    s_tmp_1 *= 2.0f;
    vec3_mul(vec3_u, s_tmp_1, vec3_tmp_1);

    s_tmp_1 = vec4_q[0];

    s_tmp_2 = vec3_dot(vec3_u, vec3_u);
    s_tmp_2 = -(s_tmp_2);
    s_tmp_2 += s_tmp_1 * s_tmp_1;
    vec3_mul(vec3_v, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_tmp_1);

    s_tmp_2 = 2.0f * s_tmp_1;
    vec3_cross(vec3_u, vec3_v, vec3_tmp_2);
    vec3_mul(vec3_tmp_2, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_out);
}

static void sleep_10ms_pd(unsigned char count) small {
    do {
        RCC_SetPowerDownWakeupTimerCountdown(0x18);
        RCC_SetPowerDownWakeupTimerState(HAL_State_ON);
        RCC_SetPowerDownMode(HAL_State_ON);
        NOP();
        NOP();
        NOP();
        NOP();
        RCC_SetPowerDownWakeupTimerState(HAL_State_OFF);
    } while (--count);
}

static void refresh_mpu(void) small {
    unsigned char more, sensors_mpu;

    do {
        mpu_read_fifo(gyro_s, NULL, &sensors_mpu, &more);
    } while (more);
}

static void refresh_dmp(void) small {
    unsigned char more;
    short sensors_mpu;
    long quat_l[4];

    do {
        dmp_read_fifo(gyro_s, NULL, quat_l, &sensors_mpu, &more);
    } while (more);

    quat_f[0] = (float)quat_l[0] / Q30;
    quat_f[1] = (float)quat_l[1] / Q30;
    quat_f[2] = (float)quat_l[2] / Q30;
    quat_f[3] = (float)quat_l[3] / Q30;
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

static unsigned char get_led_mask_by_q(void) small {
    float vec3_out[3];
    unsigned char final_mask = 0, i;

    for (i = 0; i < 8; i++) {
        rotate(ARR_VEC3_FACES_ORIENT[i], quat_f, vec3_out);
        if (IS_IN_RANGE_EPS(vec3_out[0], 0.0f, 0.2f)
            && IS_IN_RANGE_EPS(vec3_out[1], 0.0f, 0.2f)
            && IS_IN_RANGE_EPS(vec3_out[2], 1.0f, 0.2f)) {
            final_mask |= 0x01u << i;
        printf(MSG_FMT_INFO_FACE_DETECTED, (int)i, vec3_out[0], vec3_out[1], vec3_out[2]);
    }
    }

    return final_mask;
}

static void init_mpu(void) small {
    unsigned char i, who_am_i;

    printf(MSG_FMT_INFO_MPU_INIT_BEGIN);

    /* who_am_i */
    mpu_read_reg(MPU_REG_WHOAMI, &who_am_i);
    if (who_am_i != MPU_SIGNATURE_WHOAMI) {
        printf(MSG_FMT_ERR_MPU_WHOAMI_MISMATCH, (int)who_am_i, (int)MPU_SIGNATURE_WHOAMI);
        while (1) {
            set_led(MASK_LED_PYR);
            sleep_10ms_pd(100);
            set_led(0);
            sleep_10ms_pd(100);
        }
    }

    mpu_init();
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_gyro_fsr(1000);
    mpu_set_accel_fsr(8);
    mpu_set_lpf(42);
    mpu_configure_fifo(INV_XYZ_GYRO);
    mpu_set_sample_rate(MPU_REFRESH_RATE_HZ);

    for (i = 0; i < 8; i++) {
        set_led(0x01u << i);
        sleep_10ms_pd(25);
    }
    set_led(0);

    for (i = 0; i < MPU_CALIB_SAMPLES; i++) {
        refresh_mpu();
        gyro_bias_l[0] += (long)(gyro_s[0] - MPU_CALIB_GYRO_X_GOAL);
        gyro_bias_l[1] += (long)(gyro_s[1] - MPU_CALIB_GYRO_Y_GOAL);
        gyro_bias_l[2] += (long)(gyro_s[2] - MPU_CALIB_GYRO_Z_GOAL);
        sleep_10ms_pd(1);
    }
    gyro_bias_l[0] /= MPU_CALIB_SAMPLES;
    gyro_bias_l[1] /= MPU_CALIB_SAMPLES;
    gyro_bias_l[2] /= MPU_CALIB_SAMPLES;
    printf(MSG_FMT_INFO_MPU_BIAS_CALIB_DONE, (int)gyro_bias_l[0], (int)gyro_bias_l[1], (int)gyro_bias_l[2]);

    for (i = 0; i < 8; i++) {
        set_led(0x80u >> i);
        sleep_10ms_pd(25);
    }
    set_led(0);
    sleep_10ms_pd(10);

    dmp_load_motion_driver_firmware();
    dmp_set_fifo_rate(MPU_REFRESH_RATE_HZ);
    mpu_set_dmp_state(1);
    mpu_set_gyro_bias_reg(gyro_bias_l);
    dmp_set_gyro_bias(gyro_bias_l);
    dmp_enable_feature(DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_6X_LP_QUAT);
    sleep_10ms_pd(10);

    printf(MSG_FMT_INFO_MPU_INIT_END);
}

void main(void) small {
    EXTI_Global_SetIntState(HAL_State_ON);

    init_periph();
    set_led(0);
    sleep_10ms_pd(100);
    init_mpu();

    do {
        refresh_dmp();
        set_led(get_led_mask_by_q());
        sleep_10ms_pd(1);
    } while (1);
}

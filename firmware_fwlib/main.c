#include <stdio.h>
#include "fw_hal.h"
#include "inv_mpu.h"
#include "MahonyAHRS.h"

#define MPU_GYRO_FSR 2000
#define MPU_REFRESH_RATE_HZ 100
#define MPU_F_GYRO_SENS ( 16.375f )
#define MPU_F_ACCEL_SENS ( 16384.f )

int __DATA status;

short __IDATA gyro_s[3], accel_s[3];
unsigned char __IDATA more, sensors_mpu;
float __IDATA gyro_f[3], accel_f[3];

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

static void init_gpio(void) small {
    // LEDs
    GPIO_P2_SetMode(GPIO_Pin_All, GPIO_Mode_Output_PP);
    P20 = 1;
    P21 = 1;
    P22 = 1;
    P23 = 1;
    P24 = 1;
    P25 = 1;
    P26 = 1;
    P27 = 1;
    // Serial
    GPIO_P3_SetMode(GPIO_Pin_6, GPIO_Mode_Input_HIP);
    GPIO_P3_SetMode(GPIO_Pin_7, GPIO_Mode_Output_PP);
    // I2C
    GPIO_P3_SetMode(GPIO_Pin_3, GPIO_Mode_InOut_QBD);  // SDA
    GPIO_P3_SetMode(GPIO_Pin_2, GPIO_Mode_InOut_QBD);  // SCL
    GPIO_SetPullUp(GPIO_Port_3, GPIO_Pin_2 | GPIO_Pin_3, HAL_State_ON);
    GPIO_SetSwitchSpeed(GPIO_Port_3, GPIO_Pin_2 | GPIO_Pin_3, GPIO_SwitchSpeed_High);
    GPIO_SetDriveCapability(GPIO_Port_3, GPIO_Pin_2 | GPIO_Pin_3, GPIO_DriveCapability_High);
    GPIO_SetSchmittTrigger(GPIO_Port_3, GPIO_Pin_2 | GPIO_Pin_3, GPIO_SchmittTrigger_ON);
    GPIO_SetDigitalInput(GPIO_Port_3, GPIO_Pin_2 | GPIO_Pin_3, HAL_State_ON);
}

static void init_uart(void) small {
    UART1_SwitchPort(UART1_AlterPort_P36_P37);
    UART1_24M_115200_Init();
    printf("UART1: Ready\r\n");
}

static void init_i2c(void) small {
    I2C_SetWorkMode(I2C_WorkMode_Master);
    I2C_SetClockPrescaler(13U);
    I2C_SetPort(I2C_AlterPort_P32_P33);
    I2C_SetEnabled(HAL_State_ON);
}

static void init_periph_mpu(void) small {
    mpu_init(NULL);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_gyro_fsr(MPU_GYRO_FSR);
    mpu_set_accel_fsr(2);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(MPU_REFRESH_RATE_HZ);
}

void main(void) small {
    init_gpio();
    init_uart();
    init_i2c();
    init_periph_mpu();

    while (1) {
        do {
            status = mpu_read_fifo(gyro_s, accel_s, NULL, &sensors_mpu, &more);

            gyro_f[0] = ((float)gyro_s[0]) / MPU_F_GYRO_SENS;
            gyro_f[1] = ((float)gyro_s[1]) / MPU_F_GYRO_SENS;
            gyro_f[2] = ((float)gyro_s[2]) / MPU_F_GYRO_SENS;

            accel_f[0] = ((float)accel_s[0]) / MPU_F_ACCEL_SENS;
            accel_f[1] = ((float)accel_s[1]) / MPU_F_ACCEL_SENS;
            accel_f[2] = ((float)accel_s[2]) / MPU_F_ACCEL_SENS;

            MahonyAHRSUpdateIMU(gyro_f[0], gyro_f[1], gyro_f[2], accel_f[0], accel_f[1], accel_f[2]);
        } while (more);

        // printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n", 
        //     gyro_f[0], gyro_f[1], gyro_f[2],
        //     accel_f[0], accel_f[1], accel_f[2]);

        printf("%.6f,%.6f,%.6f,%.6f\r\n", q0, q1, q2, q3);

        SYS_Delay(5);
    }
}

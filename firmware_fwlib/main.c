#include <stdio.h>
#include "fw_hal.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define MPU_GYRO_FSR 2000
#define MPU_REFRESH_RATE_HZ 25
#define MPU_DMP_REFRESH_RATE_HZ 25
#define MPU_F_GYRO_SENS ( 131.0f * 250.f / (float)MPU_GYRO_FSR )
#define MPU_F_QUAT_SENS ( 1073741824.f )

unsigned char __PDATA str_buf[64];

int __DATA status;
short __IDATA gyro_s[3], sensors;
unsigned char __IDATA more;

float __DATA vec4_quat[4], gyro_f[3];

/* = = = QUATERNION AND VECTOR HELPER FUNCTIONS = = = */

static void vec3_dot(float *vec3_x, float *vec3_y, float *s_out) small {
    *s_out = vec3_x[0] * vec3_y[0]
           + vec3_x[1] * vec3_y[1]
           + vec3_x[2] * vec3_y[2]
           + vec3_x[3] * vec3_y[3];
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

    vec3_dot(vec3_u, vec3_v, &s_tmp_1);
    s_tmp_1 *= 2.f;
    vec3_mul(vec3_u, s_tmp_1, vec3_tmp_1);

    s_tmp_1 = vec4_q[0];

    vec3_dot(vec3_u, vec3_u, &s_tmp_2);
    s_tmp_2 = -(s_tmp_2);
    s_tmp_2 += s_tmp_1 * s_tmp_1;
    vec3_mul(vec3_v, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_tmp_1);

    s_tmp_2 = 2.f * s_tmp_1;
    vec3_cross(vec3_u, vec3_v, vec3_tmp_2);
    vec3_mul(vec3_tmp_2, s_tmp_2, vec3_tmp_2);

    vec3_add(vec3_tmp_1, vec3_tmp_2, vec3_out);
}

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
    UART1_Config8bitUart(UART1_BaudSource_Timer1, HAL_State_ON, 115200ul);
    printf("UART1: Ready\r\n");
}

static void init_i2c(void) small {
    I2C_SetWorkMode(I2C_WorkMode_Master);
    I2C_SetClockPrescaler(58U);
    I2C_SetPort(I2C_AlterPort_P32_P33);
    I2C_SetEnabled(HAL_State_ON);
}

static void init_periph_mpu(void) small {
    status = mpu_init(NULL);
    printf("mpu_init: %d\r\n", status);
    status = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    printf("mpu_set_sensors: %d\r\n", status);
    status = mpu_set_gyro_fsr(MPU_GYRO_FSR);
    printf("mpu_set_gyro_fsr: %d\r\n", status);
    status = mpu_set_accel_fsr(2);
    printf("mpu_set_accel_fsr: %d\r\n", status);
    status = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    printf("mpu_configure_fifo: %d\r\n", status);
    mpu_get_power_state(&status);
    printf("mpu_get_power_state: %#hhx\r\n", (unsigned short)status);
    status = mpu_set_sample_rate(MPU_REFRESH_RATE_HZ);
    printf("mpu_set_sample_rate: %d\r\n", status);
}

static void init_periph_dmp(void) small {
    status = dmp_load_motion_driver_firmware();
    printf("dmp_load_motion_driver_firmware: %d\r\n", status);
    if (status) while (1) { ; }
    status = dmp_set_fifo_rate(MPU_DMP_REFRESH_RATE_HZ);
    printf("dmp_set_fifo_rate: %d\r\n", status);
    status = dmp_enable_feature(DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    printf("dmp_enable_feature: %d\r\n", status);
    status = mpu_set_dmp_state(1);
    printf("mpu_set_dmp_state: %d\r\n", status);
}

void main(void) compact {
    init_gpio();
    init_uart();
    init_i2c();

    init_periph_mpu();
    init_periph_dmp();

    while (1) {
        status = dmp_read_fifo(gyro_s, NULL, NULL, NULL, &sensors, &more);

        gyro_f[0] = (float)gyro_s[0] / MPU_F_GYRO_SENS;
        gyro_f[1] = (float)gyro_s[1] / MPU_F_GYRO_SENS;
        gyro_f[2] = (float)gyro_s[2] / MPU_F_GYRO_SENS;

        printf("(%d, %u, %u) gx:%.3f, gy:%.3f, gz:%.3f\r\n", 
            status, sensors, more, gyro_f[0], gyro_f[1], gyro_f[2]);

        SYS_DelayUs(5);
    }
}

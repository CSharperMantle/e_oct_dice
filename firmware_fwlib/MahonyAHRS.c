//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define SAMPLE_FREQ 200.0f          // sample frequency in Hz
#define TWO_K_P_DEF (2.0f * 0.5f)   // 2 * proportional gain
#define TWO_K_I_DEF (2.0f * 0.01f)   // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = TWO_K_P_DEF;                                         // 2 * proportional gain (Kp)
volatile float twoKi = TWO_K_I_DEF;                                         // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                  // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

union float_or_long_u {
    float f;
    long l;
};

float invSqrt(float x) small {
    float halfx = 0.5f * x;
    union float_or_long_u y;
    y.f = x;
    y.l = 0x5f3759df - (y.l >> 1);
    y.f = y.f * (1.5f - (halfx * y.f * y.f));
    return y.f;
}

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) small {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;        

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
    
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / SAMPLE_FREQ);   // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / SAMPLE_FREQ);
            integralFBz += twoKi * halfez * (1.0f / SAMPLE_FREQ);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / SAMPLE_FREQ));        // pre-multiply common factors
    gy *= (0.5f * (1.0f / SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / SAMPLE_FREQ));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//====================================================================================================
// END OF CODE
//====================================================================================================

/*
 * imu_fusion.c
 *
 *  Created on: Mar 25, 2025
 *      Author: arshd
 */


#include "imu_fusion.h"
#include <math.h>
#include <string.h>

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

// EKF instance
static ekf_t ekf;

// Covariance and noise matrices
static _float_t Pdiag[EKF_N] = {0.01f, 0.01f};
static _float_t Q[EKF_N * EKF_N] = {
    0.001f, 0.0f,
    0.0f,   0.001f
};
static _float_t R[EKF_M * EKF_M] = {
    0.03f, 0.0f, 0.0f,
    0.0f, 0.03f, 0.0f,
    0.0f, 0.0f, 0.03f
};



/**
 * @brief Sensor model function.
 *
 * This function computes the predicted accelerometer measurements (hx) due to gravity
 * given the current roll (phi) and pitch (theta) estimates, and calculates the Jacobian matrix (H)
 * of the measurement function.
 *
 * The model used is:
 *    ax = -g * sin(theta)
 *    ay =  g * sin(phi) * cos(theta)
 *    az =  g * cos(phi) * cos(theta)
 *
 * The Jacobian matrix H is computed as:
 *
 *  [  0,             -g*cos(theta)             ]
 *  [ g*cos(phi)*cos(theta),   -g*sin(phi)*sin(theta) ]
 *  [ -g*sin(phi)*cos(theta),  -g*cos(phi)*sin(theta) ]
 *
 * @param ekf Pointer to the current EKF state.
 * @param hx  Output predicted measurement vector (length EKF_M).
 * @param H   Output Jacobian matrix (EKF_M x EKF_N).
 */
static void sensor_model(const ekf_t * ekf, _float_t hx[EKF_M], _float_t H[EKF_M * EKF_N])
{
    // Extract current state estimates (in radians)
    _float_t phi   = ekf->x[0];  // roll
    _float_t theta = ekf->x[1];  // pitch
    const _float_t g = 9.81f;

    // Predicted measurements:
    hx[0] = -g * sin(theta);            // ax
    hx[1] =  g * sin(phi) * cos(theta);   // ay
    hx[2] =  g * cos(phi) * cos(theta);   // az

    // Compute the Jacobian H = d[hx]/d[x]
    // For ax = -g*sin(theta)
    H[0 * EKF_N + 0] = 0.0f;              // ∂ax/∂phi
    H[0 * EKF_N + 1] = -g * cos(theta);    // ∂ax/∂theta

    // For ay = g*sin(phi)*cos(theta)
    H[1 * EKF_N + 0] = g * cos(phi) * cos(theta);   // ∂ay/∂phi
    H[1 * EKF_N + 1] = -g * sin(phi) * sin(theta);    // ∂ay/∂theta

    // For az = g*cos(phi)*cos(theta)
    H[2 * EKF_N + 0] = -g * sin(phi) * cos(theta);    // ∂az/∂phi
    H[2 * EKF_N + 1] = -g * cos(phi) * sin(theta);    // ∂az/∂theta
}

// Initialize EKF fusion
void imu_fusion_init(void)
{
    ekf_initialize(&ekf, Pdiag);
}

// Update EKF fusion state
void imu_fusion_update(float Gx, float Gy, float Gz, float Ax, float Ay, float Az, float dt,
                       float *roll_deg, float *pitch_deg)
{
    // Gyro data converted to rad/s
    float p_rps = Gx * DEG_TO_RAD;
    float q_rps = Gy * DEG_TO_RAD;

    // Prediction step
    _float_t fx[EKF_N] = {
        ekf.x[0] + p_rps * dt,
        ekf.x[1] + q_rps * dt
    };

    _float_t F[EKF_N * EKF_N] = {
        1.0f, 0.0f,
        0.0f, 1.0f
    };

    ekf_predict(&ekf, fx, F, Q);

    // Measurement update
    _float_t z[EKF_M] = { Ax, Ay, Az };
    _float_t hx[EKF_M];
    _float_t H[EKF_M * EKF_N];
    sensor_model(&ekf, hx, H);

    ekf_update(&ekf, z, hx, H, R);

    // Export roll and pitch in degrees
    *roll_deg = ekf.x[0] * RAD_TO_DEG;
    *pitch_deg = ekf.x[1] * RAD_TO_DEG;
}


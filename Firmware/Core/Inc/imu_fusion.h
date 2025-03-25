/*
 * imu_fusion.h
 *
 *  Created on: Mar 25, 2025
 *      Author: arshd
 */

#ifndef INC_IMU_FUSION_H_
#define INC_IMU_FUSION_H_

// Define the dimensions for the state and measurement vectors:
#define EKF_N 2      // State: [roll, pitch]
#define EKF_M 3      // Measurement: [ax, ay, az]


#include "tinyekf.h"  // This header uses EKF_N and EKF_M macros

// Initialize EKF fusion module
void imu_fusion_init(void);


// Update EKF state with gyro (deg/s) and accelerometer data (m/s²), returns roll and pitch in degrees
void imu_fusion_update(float Gx, float Gy, float Gz, float Ax, float Ay, float Az, float dt,
                       float *roll_deg, float *pitch_deg);


#endif /* INC_IMU_FUSION_H_ */

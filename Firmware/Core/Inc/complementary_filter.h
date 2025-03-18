/*
 * complementary_filter.h
 *
 *  Created on: Mar 18, 2025
 *      Author: arshd
 */

#ifndef INC_COMPLEMENTARY_FILTER_H_
#define INC_COMPLEMENTARY_FILTER_H_

#include <math.h>

// Define filter coefficient and conversion constants
#define COMP_FILT_ALPHA 0.0500000000f
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)
#define G_MPS2 9.8100000000f


// Function prototype:
// Provide accelerometer data (Ax, Ay, Az in g units) and gyro data (Gx, Gy, Gz in deg/s)
// dt is the sampling interval in seconds.
// The function outputs the estimated roll and pitch angles (in degrees) via pointers.
void updateComplementaryFilter(float Ax, float Ay, float Az,
                               float Gx, float Gy, float Gz,
                               float dt,
                               float *roll_deg, float *pitch_deg);

#endif /* INC_COMPLEMENTARY_FILTER_H_ */

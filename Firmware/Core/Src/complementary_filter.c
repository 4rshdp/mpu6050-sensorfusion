/*
 * complementary_filter.c
 *
 *  Created on: Mar 18, 2025
 *      Author: arshd
 */


#include "complementary_filter.h"

// Static variables to hold the current estimates in radians
static float roll_estimate_rad = 0.0f;
static float pitch_estimate_rad = 0.0f;

void updateComplementaryFilter(float Ax, float Ay, float Az,
                               float Gx, float Gy, float Gz,
                               float dt,
                               float *roll_deg, float *pitch_deg)
{

  /*
   * Calculate roll and pitch from accelerometer readings:
   *
   * Roll (φ):
   *   - Uses atan2f(Ay, Az) to compute the angle of tilt around the X-axis.
   *   - This method correctly handles quadrant determination and cases where Az is near zero.
   *
   * Pitch (θ):
   *   - Uses atan2f(-Ax, sqrt(Ay * Ay + Az * Az)) to calculate the tilt around the Y-axis.
   *   - The denominator sqrt(Ay² + Az²) accounts for gravity's distribution on the Y and Z axes,
   *     providing a more balanced pitch value.
   *
   * Both angles are then converted from radians to degrees.
   */

    // Compute accelerometer-based angles:
    float phiHat_acc_rad   = atan2f(Ay, Az);
    float thetaHat_acc_rad = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));

    // Convert gyro values from deg/s to rad/s
    float p_rps = Gx * DEG_TO_RAD;
    float q_rps = Gy * DEG_TO_RAD;
    float r_rps = Gz * DEG_TO_RAD;

    // Calculate Euler angle derivatives from gyro rates:
    float phiDot_rps   = p_rps + tanf(pitch_estimate_rad) * (sinf(roll_estimate_rad) * q_rps + cosf(roll_estimate_rad) * r_rps);
    float thetaDot_rps = cosf(roll_estimate_rad) * q_rps - sinf(roll_estimate_rad) * r_rps;

    // Integrate gyro rates and combine with accelerometer estimates using complementary filter:
    roll_estimate_rad  = COMP_FILT_ALPHA * phiHat_acc_rad   + (1.0f - COMP_FILT_ALPHA) * (roll_estimate_rad  + dt * phiDot_rps);
    pitch_estimate_rad = COMP_FILT_ALPHA * thetaHat_acc_rad + (1.0f - COMP_FILT_ALPHA) * (pitch_estimate_rad + dt * thetaDot_rps);

    // Convert the estimates to degrees:
    *roll_deg  = roll_estimate_rad  * RAD_TO_DEG;
    *pitch_deg = pitch_estimate_rad * RAD_TO_DEG;
}



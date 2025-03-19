/*
 * mpu6000.h
 *
 *  Created on: Mar 3, 2025
 *      Author: arshd
 */

/*



/**
 *@brief Concise overview of MPU6050 sensor ranges and usage
 *
 * Gyroscope full-scale range: ±250, ±500, ±1000, ±2000 °/s
 * Accelerometer full-scale range: ±2g, ±4g, ±8g, ±16g
 *
 * - Narrow range (e.g., ±2g, ±250°/s):
 * - Higher resolution for small motions
 *   - Easy to saturate under high acceleration/rotation
 *
 * - Wide range (e.g., ±16g, ±2000°/s):
 * - Handles larger/faster motions without clipping
 * - Lower resolution for subtle changes
 *
 *
 * - Gyroscope raw readings: range −32768 to +32767, representing angular velocity
 * - Accelerometer raw readings: range −32768 to +32767, representing linear acceleration
 *
 *
 * A "scale factor" (LSB per unit) is tied to each configured full-scale range.
 * For the accelerometer:
 *  - ±2g range → 16,384 LSB per g
 *  - ±4g range → 8,192 LSB per g
 *  - ±8g range → 4,096 LSB per g
 *  - ±16g range → 2,048 LSB per g
 *
 * For the gyroscope:
 *  - ±250°/s → 131 LSB per °/s
 *  - ±500°/s → 65.5 LSB per °/s
 *  - ±1000°/s → 32.8 LSB per °/s
 *  - ±2000°/s → 16.4 LSB per °/s
 *
 */

/**
 * @brief MPU6050 raw values and unit conversion
 *
 * - **Gyroscope (°/s)**: Measures angular velocity.
 *   - Formula: `AngularVelocity_dps = RawValue / ScaleFactor`
 *   - Example (±250°/s range, ScaleFactor = 131 LSB/°/s):
 *   - Raw 131 → 131 / 131 = **1°/s**
 *
 * - **Accelerometer (g)**: Measures linear acceleration.
 *   - Formula: `Acceleration_g = RawValue / ScaleFactor`
 *   - Example (±2g range, ScaleFactor = 16,384 LSB/g):
 *   - Raw 16,384 → 16,384 / 16,384 = **1g**
 *
 */



#ifndef INC_MPU6000_H_
#define INC_MPU6000_H_

#include "stm32f4xx_hal.h"

#define mpu6050           0x68
#define mpu6050addr       mpu6050 << 1
#define whoAmIReg         0x75 // to read 0x68 is exists
#define powerManagmentReg 0x6B
#define sampleRateDiv 	  0x19 // sampleRate = gyroRate / (1 + sampleDiv)
#define gyroConf		  0x1B
#define accelConf		  0x1C

#define accelMeasure      0x3B
#define gyroMeasure       0x43


#define validCondition1 (whoAreYou == mpu6050)

extern I2C_HandleTypeDef hi2c1;

extern uint8_t whoAreYou;
extern uint8_t MemData;

typedef enum{
	degS250  = 0,
	degS500  = 1,
	degS1000 = 2,
	degS2000 = 3
}gyroScale_t;

typedef enum{
	g2  = 0,
	g4  = 1,
	g8  = 2,
	g16 = 3
}accelScale_t;

typedef enum {
    DLPF_CFG_260HZ = 0,  // Accelerometer: 260 Hz (Fs = 1kHz), Gyroscope: 256 Hz, Delay: ~0.98 ms
    DLPF_CFG_184HZ = 1,  // Accelerometer: 184 Hz, Gyroscope: 188 Hz, Delay: ~2.0 ms
    DLPF_CFG_94HZ  = 2,  // Accelerometer: 94 Hz,  Gyroscope: 98 Hz,  Delay: ~3.0 ms
    DLPF_CFG_44HZ  = 3,  // Accelerometer: 44 Hz,  Gyroscope: 42 Hz,  Delay: ~4.9 ms
    DLPF_CFG_21HZ  = 4,  // Accelerometer: 21 Hz,  Gyroscope: 20 Hz,  Delay: ~8.5 ms
    DLPF_CFG_10HZ  = 5,  // Accelerometer: 10 Hz,  Gyroscope: 10 Hz,  Delay: ~13.8 ms
    DLPF_CFG_5HZ   = 6,  // Accelerometer: 5 Hz,   Gyroscope: 5 Hz,   Delay: ~19.0 ms
    DLPF_CFG_RESERVED = 7
} dlpfCfg_t;


extern float Ax, Ay, Az;
extern float Gx, Gy, Gz;

void mpu6050Config(void);
void mpu6050Init(void);
void mpu6050powerOn(void);
void mpu6050Sampling(void);
void mpu6050GyroScale(gyroScale_t scale);
void mpu6050AccelScale(accelScale_t scale);
void mpu6050ConfigFilter(uint8_t ext_sync, dlpfCfg_t dlpf_cfg);

void mpu6050GyroRead(void);
void mpu6050AccelRead(void);

void mpu6050Read_DMA(void);

#endif /* INC_MPU6000_H_ */

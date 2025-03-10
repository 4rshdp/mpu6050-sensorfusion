/*
 * mpu6000.c
 *
 *  Created on: Mar 3, 2025
 *      Author: arshd
 */

#include "mpu6000.h"

uint8_t whoAreYou     = 0;
uint8_t MemData       = 0;

int16_t RAWgyroX      = 0;
int16_t RAWgyroY      = 0;
int16_t RAWgyroZ      = 0;

int16_t RAWaccelX     = 0;
int16_t RAWaccelY     = 0;
int16_t RAWaccelZ     = 0;

float Ax = 0.0f, Ay = 0.0f, Az = 0.0f;
float Gx = 0.0f, Gy = 0.0f, Gz = 0.0f;
float temperature = 0.0f;

uint8_t mpu6050Data[14];




void mpu6050Init(void){
	HAL_I2C_Mem_Read(
				&hi2c1,
				mpu6050addr,
				whoAmIReg,
				1,
				&whoAreYou,
				1,
				100
				);
}

void mpu6050powerOn(void){
	MemData = 0x00;
	HAL_I2C_Mem_Write(
			&hi2c1,
			mpu6050addr,
			powerManagmentReg,
			1,
			&MemData,
			1,
			100
			);
}

void mpu6050Sampling(void){
	MemData = 0x07;
	HAL_I2C_Mem_Write(
			&hi2c1,
			mpu6050addr,
			sampleRateDiv,
			1,
			&MemData,
			1,
			100
			);
}

void mpu6050GyroScale(gyroScale_t scale){
	MemData = 0x00 | (scale << 3);
		HAL_I2C_Mem_Write(
			&hi2c1,
			mpu6050addr,
			gyroConf,
			1,
			&MemData,
			1,
			100
			);
}

void mpu6050AccelScale(accelScale_t scale){
	MemData = 0x00 | (scale << 3);
		HAL_I2C_Mem_Write(
			&hi2c1,
			mpu6050addr,
			accelConf,
			1,
			&MemData,
			1,
			100
			);
}

void mpu6050Config(void){
	// is valid Condition true 0x68
	mpu6050Init();

	if(validCondition1){
	// power on
		mpu6050powerOn();
	// sampling data ratio
		mpu6050Sampling();
	// gyro scale   (RAW)
		mpu6050GyroScale(degS2000);
	// accel scale  (RAW)
		mpu6050AccelScale(g16);
	}
}




void mpu6050GyroRead(void){
	uint8_t gyroData[6];
	HAL_I2C_Mem_Read(
			&hi2c1,
			mpu6050addr,
			gyroMeasure,
			1,
			gyroData,
			6,
			100
			);

	RAWgyroX = (uint16_t) (gyroData[0] << 8 | gyroData[1]);
	RAWgyroY = (uint16_t) (gyroData[2] << 8 | gyroData[3]);
	RAWgyroZ = (uint16_t) (gyroData[4] << 8 | gyroData[5]);

	Gx = RAWgyroX/16.4;
	Gy = RAWgyroY/16.4;
	Gz = RAWgyroZ/16.4;
}

void mpu6050AccelRead(void){
	uint8_t accelData[6];
	HAL_I2C_Mem_Read(
			&hi2c1,
			mpu6050addr,
			accelMeasure,
			1,
			accelData,
			6,
			100
			);

	RAWaccelX = (uint16_t) (accelData[0] << 8 | accelData[1]);
	RAWaccelY = (uint16_t) (accelData[2] << 8 | accelData[3]);
	RAWaccelZ = (uint16_t) (accelData[4] << 8 | accelData[5]);

	Ax = RAWaccelX/2048.0;
	Ay = RAWaccelY/2048.0;
	Az = RAWaccelZ/2048.0;
}


void mpu6050Read_DMA(void) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1,
                                                    mpu6050addr,
                                                    0x3B,       // Starting register address
                                                    1,          // Memory address size (1 byte)
                                                    mpu6050Data,// Buffer to store the 14 bytes
                                                    14);        // Number of bytes to read
    if(status != HAL_OK) {
        // Handle error here, e.g., log or trigger an error flag.
    }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if(hi2c->Instance == I2C1) {
        // Accelerometer data conversion
        int16_t rawAccelX = (int16_t)((mpu6050Data[0] << 8) | mpu6050Data[1]);
        int16_t rawAccelY = (int16_t)((mpu6050Data[2] << 8) | mpu6050Data[3]);
        int16_t rawAccelZ = (int16_t)((mpu6050Data[4] << 8) | mpu6050Data[5]);

        // Temperature data conversion
        int16_t rawTemp = (int16_t)((mpu6050Data[6] << 8) | mpu6050Data[7]);
        // Conversion formula example: actualTemp = (rawTemp / 340.0) + 36.53

        // Gyro data conversion
        int16_t rawGyroX = (int16_t)((mpu6050Data[8]  << 8) | mpu6050Data[9]);
        int16_t rawGyroY = (int16_t)((mpu6050Data[10] << 8) | mpu6050Data[11]);
        int16_t rawGyroZ = (int16_t)((mpu6050Data[12] << 8) | mpu6050Data[13]);


        Ax = rawAccelX / 2048.0f;
        Ay = rawAccelY / 2048.0f;
        Az = rawAccelZ / 2048.0f;

        temperature = (rawTemp / 340.0f) + 36.53f; // Example conversion formula

        Gx = rawGyroX / 16.4f;
        Gy = rawGyroY / 16.4f;
        Gz = rawGyroZ / 16.4f;

        // Process or store these values as needed
        // For example, update global variables or send them to another module

        // If you need continuous reading and are not using circular mode,
        // you can re-initiate the DMA read here.
        mpu6050Read_DMA();
    }
}








/*
 * MPU9500.h
 *
 *  Created on: Nov 10, 2023
 *      Author: panna
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "main.h"
#include "MPU9250_RegisterMap.h"

typedef struct MPU9250{
	float AccelX;
	float AccelY;
	float AccelZ;

	float GyroX;
	float GyroY;
	float GyroZ;

	float MagnX;
	float MagnY;
	float MagnZ;

	float roll;
	float pitch;
	float yaw;
}MPU9250;

void GY6500_init(I2C_HandleTypeDef *hi2c);
void MPU9250_init(I2C_HandleTypeDef *hi2c);
void AK8963_init(I2C_HandleTypeDef *hi2c, float *data);

void MPU9250_Calibrate(I2C_HandleTypeDef *hi2c, float *data1, float *data2);
void AK8963_Calibrate(I2C_HandleTypeDef *hi2c, float *data1, float *data2);

void setGyroRes();
void setAccelRes();
void setMagRes();

void readGyro(I2C_HandleTypeDef *hi2c, int16_t *Data);
void readAccel(I2C_HandleTypeDef *hi2c, int16_t *Data);
void readMag(I2C_HandleTypeDef *hi2c, int16_t *Data);

void allRead(I2C_HandleTypeDef *hi2c, MPU9250 *DataStruct);

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
#endif /* INC_MPU9250_H_ */

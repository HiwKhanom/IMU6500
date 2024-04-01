/*
 * MPU9500.c
 *
 *  Created on: Mar 30, 2024
 *      Author: panna
 */
#include "main.h"
#include "math.h"
#include "MPU9250.h"
#include "MPU9250_RegisterMap.h"

#define I2C_Delay 		100
#define I2C_Timeout  	50

uint8_t Gscale = GFS_2000DPS;
uint8_t Ascale = AFS_16G;
uint8_t Mscale = MFS_16BITS;
uint8_t Mmode = 0x06;

float gyroRes, accelRes, magRes;

int16_t gyroOut[3];
int16_t accelOut[3];
int16_t magOut[3];

float gyrobias[3] = {0, 0, 0};
float accelbias[3] = {0, 0, 0};

float magBias[3]= {0, 0, 0};
float magScale[3]= {0, 0, 0};
float magCalibration[3]= {0, 0, 0};


float beta = 0.6045998;
float zeta = 0.0;

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float pitch, yaw, roll;
float deltaT = 0.0;
float sum = 0.0;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;
uint32_t firstUpdate = 0; // used to calculate integration interval
uint32_t now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components

void GY6500_init(I2C_HandleTypeDef *hi2c){
	uint8_t readBuffer;
	uint8_t writeBuffer;

	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);  //MPU9250 should return 0x71 back is everything was fine

	if(readBuffer == 0x71){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);

		//Calibrate gyro and accelrometers, load biases in bias registers
		MPU9250_Calibrate(hi2c, gyrobias, accelbias);
		HAL_Delay(1000);

		//init Gyro and Accelerometer
		MPU9250_init(hi2c);

		//enable Mag bypass
		writeBuffer = 0x22;
		HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

		//Get magnetometer calibration from AK8963 ROM
		AK8963_init(hi2c, magCalibration);  // Initialize device for active mode read of magnetometer

		AK8963_Calibrate(hi2c, magBias, magScale);
		HAL_Delay(1000);

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(150);
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(150);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	}

	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	}
}

void allRead(I2C_HandleTypeDef *hi2c, MPU9250 *DataStruct){
	uint8_t readBuffer;

	// If intPin goes high, all data registers have new data
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, INT_STATUS, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	if (readBuffer & 0x01) {  // On interrupt, check if data ready interrupt
		setAccelRes();
		readAccel(hi2c, accelOut);

		ax = (float)accelOut[0] * accelRes;
		ay = (float)accelOut[1] * accelRes;
		az = (float)accelOut[2] * accelRes;

		DataStruct->AccelX = ax;
		DataStruct->AccelY = ay;
		DataStruct->AccelZ = az;

		setGyroRes();
		readGyro(hi2c, gyroOut);

		gx = (float)gyroOut[0] * gyroRes;
		gy = (float)gyroOut[1] * gyroRes;
		gz = (float)gyroOut[2] * gyroRes;

		DataStruct->GyroX = gx;
		DataStruct->GyroY = gy;
		DataStruct->GyroZ = gz;

		setMagRes();
		readMag(hi2c, magOut);
		mx = (float)magOut[0] * magRes * magCalibration[0] - magBias[0];
		my = (float)magOut[1] * magRes * magCalibration[1] - magBias[1];
		mz = (float)magOut[2] * magRes * magCalibration[2] - magBias[2];
		mx *= magScale[0];
		my *= magScale[1];
		mz *= magScale[2];

		DataStruct->MagnX = mx;
		DataStruct->MagnY = my;
		DataStruct->MagnZ = mz;
	}

	now = HAL_GetTick();
	deltaT = ((now - lastUpdate) / 1000.0); // set integration time by time elapsed since last filter update
	lastUpdate = now;
	sum += deltaT; // sum for averaging filter update rate

	// Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro.

	// Calculate quaternions based on Madgwick's filter
	//Since MPU9250's mag. and IMU modules are different and seperate (AK8963 and MPU6050), their...
	//...coordinate systems also different. So, to compensate this, order should be my - mx - mz
	//QuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz);
	QuaternionUpdate(ax, ay, az, gx * M_PI / 180.0f, gy * M_PI / 180.0f, gz * M_PI / 180.0f,  my,  mx, mz);

	// Convert quaternions to Euler angles
	a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
	a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
	a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
	a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

	pitch = -asinf(a32);
	roll  = atan2f(a31, a33);
	yaw   = atan2f(a12, a22);
	pitch *= 180.0f / M_PI;
	yaw   *= 180.0f / M_PI;
	yaw   += 5.53f; // Declination

	if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
	roll  *= 180.0f / M_PI;
	lin_ax = ax + a31;
	lin_ay = ay + a32;
	lin_az = az - a33;

	DataStruct->yaw = yaw;
	DataStruct->pitch = pitch;
	DataStruct->roll = roll;

	sum = 0;
}

void setMagRes(){
	switch(Mscale){
		case MFS_14BITS :
			magRes = (10.0 * 4912.0) / 8190.0;
			break;
		case MFS_16BITS :
			magRes = (10.0 * 4912.0) / 32768.0;
		}
}

void setGyroRes(){
	switch(Gscale){
		case GFS_250DPS :
			gyroRes = 250.0/32768.0;
			break;
		case GFS_500DPS :
			gyroRes = 500.0/32768.0;
			break;
		case GFS_1000DPS :
			gyroRes = 1000.0/32768.0;
			break;
		case GFS_2000DPS :
			gyroRes = 2000.0/32768.0;
			break;
	}
}

void setAccelRes(){
	switch(Ascale){
		case AFS_2G :
			accelRes = 2.0/32768.0;
			break;
		case AFS_4G :
			accelRes = 4.0/32768.0;
			break;
		case AFS_8G :
			accelRes = 8.0/32768.0;
			break;
		case AFS_16G :
			accelRes = 16.0/32768.0;
			break;
	}
}

void readAccel(I2C_HandleTypeDef *hi2c, int16_t *Data){
	uint8_t readAccelBuffer[6];

	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, &readAccelBuffer[0], 6, I2C_Timeout);
	Data[0] = ((int16_t)readAccelBuffer[0] << 8 | readAccelBuffer[1]);
	Data[1] = ((int16_t)readAccelBuffer[2] << 8 | readAccelBuffer[3]);
	Data[2] = ((int16_t)readAccelBuffer[4] << 8 | readAccelBuffer[5]);
}

void readGyro(I2C_HandleTypeDef *hi2c, int16_t *Data){
	uint8_t readGyroBuffer[6];
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &readGyroBuffer[0], 6, I2C_Timeout);
	Data[0] = ((int16_t)readGyroBuffer[0] << 8 | readGyroBuffer[1]);
	Data[1] = ((int16_t)readGyroBuffer[2] << 8 | readGyroBuffer[3]);
	Data[2] = ((int16_t)readGyroBuffer[4] << 8 | readGyroBuffer[5]);
}

void readMag(I2C_HandleTypeDef *hi2c, int16_t *Data){
	uint8_t readStatus;

	//check for data if data is ready
	HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &readStatus, 1, I2C_Timeout);
	if((readStatus & 0x01) == 0x01){
		uint8_t readBuffer[7];
		//read the data of magnetometer and ST2 for overflow data status
		HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, &readBuffer[0], 7, I2C_Timeout);

		//check if data is overflow
		uint8_t preCheck = readBuffer[6];
		if(!(preCheck & 0x08)){
			Data[0] = ((int16_t)readBuffer[1] << 8) | readBuffer[0];
			Data[1] = ((int16_t)readBuffer[3] << 8) | readBuffer[2];
			Data[2] = ((int16_t)readBuffer[5] << 8) | readBuffer[4];
		}
	}
}

void AK8963_init(I2C_HandleTypeDef *hi2c, float *data){
	uint8_t writeBuffer;
	uint8_t readBuffer[3];

	//Push value from ROM for megnetometer configuration
	//Turn off megnetometer
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//Turn off ROM access mode
	writeBuffer = 0x0F;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//read x, y, z axis calibration value from factory
	HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS, AK8963_ASAX, I2C_MEMADD_SIZE_8BIT, &readBuffer[0], 3, I2C_Timeout);
	//Cal sensitivity adjustment
	data[0] = (float)(readBuffer[0] - 128)/256.0 + 1.0;
	data[1] = (float)(readBuffer[1] - 128)/256.0 + 1.0;
	data[2] = (float)(readBuffer[2] - 128)/256.0 + 1.0;

	//Turn off megnetometer for configuration
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//Config magnetometer to start reading value and set to high resolution
	writeBuffer = Mscale << 4 | Mmode;
	HAL_I2C_Mem_Write(hi2c, AK8963_ADDRESS, AK8963_CNTL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(10);
}

void AK8963_Calibrate(I2C_HandleTypeDef *hi2c, float *data1, float *data2){
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	// shoot for ~fifteen seconds of mag data
	if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	for(ii = 0; ii < sample_count; ii++) {
		readMag(hi2c, mag_temp);  // Read the mag data

		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}

		if(Mmode == 0x02) HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if(Mmode == 0x06) HAL_Delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
	}

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	data1[0] = (float) mag_bias[0]*magRes*magCalibration[0];  // save mag biases in G for main program
	data1[1] = (float) mag_bias[1]*magRes*magCalibration[1];
	data1[2] = (float) mag_bias[2]*magRes*magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	data2[0] = avg_rad/((float)mag_scale[0]);
	data2[1] = avg_rad/((float)mag_scale[1]);
	data2[2] = avg_rad/((float)mag_scale[2]);
}

void MPU9250_init(I2C_HandleTypeDef *hi2c){
	uint8_t readBuffer;
	uint8_t writeBuffer;

	//wake sensor up
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//set oscillator to auto select suitable clk source, else use 20MHz internal
	writeBuffer = 0x01;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//--------------------------------------------//
	//MPU9250 Datasheet v1.4 Page13
	//set Digital low pass filter (DLPF) in DLPF_CFG bit
	/*
			  FCHOICE          |   DLPF_CFG   |                  Gyroscope             |     Temperature Sensor
		 <1>     |     <0>                      Bandwidth (Hz) | Delay (ms) | Fs (kHz)   Bandwidth (Hz) | Delay (ms)
	-----------------------------------------------------------------------------------------------------------------
		  x      |      0      |      x       |      8800      |    0.064   |    32    |      4000      |    0.04
		  0      |      1      |      x       |      3600      |    0.11    |    32    |      4000      |    0.04
		  1      |      1      |      0       |      250       |    0.97    |    8     |      4000      |    0.04
		  1      |      1      |      1       |      184       |    2.9     |    1     |      188       |    1.9
		  1      |      1      |      2       |      92        |    3.9     |    1     |      98        |    2.8
		  1      |      1      |      3       |      41        |    5.9     |    1     |      42        |    4.8
		  1      |      1      |      4       |      20        |    9.9     |    1     |      20        |    8.3
		  1      |      1      |      5       |      10        |    17.85   |    1     |      10        |    13.4
		  1      |      1      |      6       |      5         |    33.48   |    1     |      5         |    18.6
		  1      |      1      |      7       |      3600      |    0.17    |    8     |      4000      |    0.04

	Note : Fs is Internal Sampling rate
	*/
	//set Internal Sampling rate to 8 kHz
	writeBuffer = 0x03;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, CONFIG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//set Ourput Rate to 1 kHz
	//Note from datasheet Page 12 : Data should be sampled at or above sample rate; SMPLRT_DIV is only used for1kHz internal sampling.
	writeBuffer = 0x04;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//set GyroConfig to Gscale *default in this code 250DPS*
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	readBuffer = readBuffer & ~0x03; //Clear Fchoise bits[1:0] to 0xX0
	readBuffer = readBuffer & ~0x18; //Clear GyroScale[4:3] to 0xXX
	readBuffer = readBuffer | Gscale << 3; //Set Gyro scale
	HAL_Delay(I2C_Delay);

	writeBuffer = readBuffer;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//set AccelConfig to Ascale *default in this code is 2g*
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	readBuffer = readBuffer & ~0x18; //Clear AFS bits[4:3]
	readBuffer = readBuffer | Ascale << 3; //Set Accel scale

	writeBuffer = readBuffer;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);

	//set Accel Data Rate
	/*
															 Output
	 ACCEL_FCHOICE | A_DLPF_CFG |  Bandwidth (Hz) | Delay (ms) | Noise Density (ug/rtHz) | Rate (kHz)
	-----------------------------------------------------------------------------------------------
		   0       |      1     |       1.13      |    0.75    |           250           |      4
		   1       |      0     |       460       |    1.94    |           250           |      1
		   1       |      1     |       184       |    5.80    |           250           |      1
		   1       |      2     |       92        |    7.80    |           250           |      1
		   1       |      3     |       41        |   11.80    |           250           |      1
		   1       |      4     |       20        |   19.80    |           250           |      1
		   1       |      5     |       10        |   35.70    |           250           |      1
		   1       |      6     |       5         |   66.96    |           250           |      1
		   1       |      7     |       460       |    1.94    |           250           |      1
	*/
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, ACCEL_CONFIG_2, I2C_MEMADD_SIZE_8BIT, &readBuffer, 1, I2C_Timeout);
	readBuffer = readBuffer & ~0x0F; //Clear accel_fchoise_b[3] and A_DLPFG[2:0]
	readBuffer = readBuffer | 0x03;  //Set accelerometer rate to 1kHz and bandwidth to 41 Hz

	writeBuffer = readBuffer;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, ACCEL_CONFIG_2, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(I2C_Delay);
}

void MPU9250_Calibrate(I2C_HandleTypeDef *hi2c, float *data1, float *data2){
	uint8_t writeBuffer;

	uint8_t calibData[12]; //array data to hold accel and gyro x, y, z data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0};
	int32_t accel_bias[3] = {0, 0, 0};

	//reset device
	writeBuffer = 0x80;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(100);

	//get stable time source; Auto select clock source to be PLL gyroscpoe reference if ready
	//else use the internal oscillator, bits[2:0] = 0x01
	writeBuffer = 0x01;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	HAL_Delay(200);

	//Configure device for bias calculation
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout); //Disable all interrupt
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout); // Disable Fifo
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Turn on internal clock source
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, I2C_MST_CTRL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Disable I2C master
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Disable FIFO and I2C master modes
	writeBuffer = 0x0C;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Reset FIFO and DMP
	HAL_Delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeBuffer = 0x01;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, CONFIG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Set low-pass filter to 188 Hz
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Set sample rate to 1 kHz
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeBuffer = 0x40;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Enable FIFO
	writeBuffer = 0x78;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeBuffer = 0x00;
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, FIFO_EN, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);// Disable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, FIFO_COUNTH, I2C_MEMADD_SIZE_8BIT, &calibData[0], 2, I2C_Timeout);// read FIFO sample count
	fifo_count = ((uint16_t)calibData[0] << 8) | calibData[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, FIFO_R_W, 1, &calibData[0], 12, I2C_Timeout);

		//Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t) (((int16_t)calibData[0] << 8) | calibData[1]  ) ;
		accel_temp[1] = (int16_t) (((int16_t)calibData[2] << 8) | calibData[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)calibData[4] << 8) | calibData[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)calibData[6] << 8) | calibData[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)calibData[8] << 8) | calibData[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)calibData[10] << 8) | calibData[11]) ;

		//Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t) accel_temp[0];
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	//Normalize sums to get average count biases
	accel_bias[0] /= (int32_t) packet_count;
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	//Remove gravity from the z-axis accelerometer bias calculation
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	//Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	calibData[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	calibData[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	calibData[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	calibData[3] = (-gyro_bias[1]/4)       & 0xFF;
	calibData[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	calibData[5] = (-gyro_bias[2]/4)       & 0xFF;

	//Push gyro biases to hardware registers
	writeBuffer = calibData[0];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, XG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[1];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, XG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[2];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, YG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[3];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, YG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[4];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, ZG_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[5];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, ZG_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//Output scaled gyro biases for display in the main program
	data1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	data1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	data1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	//Construct the accelerometer biases for push to the hardware accelerometer bias registers.
	int32_t accel_bias_reg[3] = {0, 0, 0}; //A place to hold the factory accelerometer trim biases
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &calibData[0], 2, I2C_Timeout); //Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &calibData[0], 2, I2C_Timeout);
	accel_bias_reg[1] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);
	HAL_I2C_Mem_Read(hi2c, mpu9250_AD0_low, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &calibData[0], 2, I2C_Timeout);
	accel_bias_reg[2] = (int32_t) (((int16_t)calibData[0] << 8) | calibData[1]);

	//Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint32_t mask = 1uL;
	//Define array to hold mask bit for each accelerometer bias axis
	uint8_t mask_bit[3] = {0, 0, 0};

	for(ii = 0; ii < 3; ii++) {
		//If temperature compensation bit is set, record that fact in mask_bit
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01;
	}

	//Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); //Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	calibData[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	calibData[1] = (accel_bias_reg[0])      & 0xFF;
	calibData[1] = calibData[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	calibData[3] = (accel_bias_reg[1])      & 0xFF;
	calibData[3] = calibData[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	calibData[5] = (accel_bias_reg[2])      & 0xFF;
	calibData[5] = calibData[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	//Push accelerometer biases to hardware registers
	writeBuffer = calibData[0];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[1];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, XA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[2];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[3];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, YA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[4];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);
	writeBuffer = calibData[5];
	HAL_I2C_Mem_Write(hi2c, mpu9250_AD0_low, ZA_OFFSET_L, I2C_MEMADD_SIZE_8BIT, &writeBuffer, 1, I2C_Timeout);

	//Output scaled accel biases for display in the main program
	data2[0] = (float) accel_bias[0]/(float) accelsensitivity;
	data2[1] = (float) accel_bias[1]/(float) accelsensitivity;
	data2[2] = (float) accel_bias[2]/(float) accelsensitivity;

}

void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltaT;
    q2 += qDot2 * deltaT;
    q3 += qDot3 * deltaT;
    q4 += qDot4 * deltaT;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

#include <stdint.h>
#include <main.h>
#include "math.h"

float freq, dTime;

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG		0x19
#define GYRO_CONFIG_REG		0x1B
#define ACCEL_CONFIG_REG	0x1C
#define ACCEL_XOUT_H_REG	0x3B
#define TEMP_OUT_H_REG		0x41
#define GYRO_XOUT_H_REG		0x43
#define PWR_MGMT_1_REG		0x6B
#define WHO_AM_I_REG		0x75

int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;

int16_t Gyro_X_RAW;
int16_t Gyro_Y_RAW;
int16_t Gyro_Z_RAW;

double Ax, Ay, Az, Gx, Gy, Gz;
double Ax_angle, Ay_angle, Az_angle, Gx_angle, Gy_angle, Gz_angle;
double Ax_0, Ay_0, Az_0, Gx_0, Gy_0, Gz_0;

uint8_t check;
uint8_t Data;
uint8_t Rec_Data_Accel[6];
uint8_t Rec_Data_Gyro[6];

uint8_t MPU6050_Calib_status;

double pitch, roll, yaw;

void MPU6050_Init (I2C_HandleTypeDef* hi2c1);
void MPU6050_reset(I2C_HandleTypeDef* hi2c1);
void MPU6050_getStatus(I2C_HandleTypeDef* hi2c1);
void MPU6050_Read_Accel (I2C_HandleTypeDef* hi2c1);
void MPU6050_Read_Gyro (I2C_HandleTypeDef* hi2c1);
void MPU6050_Calibration(I2C_HandleTypeDef* hi2c1);

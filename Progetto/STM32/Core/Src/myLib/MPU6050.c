#include "myLib/MPU6050.h"


void MPU6050_Init (I2C_HandleTypeDef* hi2c1){
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &check, 1, 1000);
	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
		Data = 0x00;
		HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void MPU6050_reset(I2C_HandleTypeDef* hi2c1){
	// power management register 0X6B
	Data = 0x80;
	HAL_I2C_Mem_Write(hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
}

void MPU6050_getStatus(I2C_HandleTypeDef* hi2c1){
	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &check, 1, 1000);
	if (check != 104){
		MPU6050_reset(hi2c1);
		MPU6050_Init(hi2c1);
	}
}


void MPU6050_Read_Accel (I2C_HandleTypeDef* hi2c1){
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data_Accel, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data_Accel[0] << 8 | Rec_Data_Accel [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data_Accel[2] << 8 | Rec_Data_Accel [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data_Accel[4] << 8 | Rec_Data_Accel [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = (double)(Accel_X_RAW/16384.0);
	Ay = (double)(Accel_Y_RAW/16384.0);
	Az = (double)(Accel_Z_RAW/16384.0);

	if(MPU6050_Calib_status == 1){
//		Ax = (double)((Ax - Ax_0)* 180 / M_PI);
//		Ay = (double)((Ay - Ay_0)* 180 / M_PI);
//		Az = (double)((Az - Az_0)* 180 / M_PI);

		roll	=	(double)( (atan2(Ay, sqrt(Az*Az+Ax*Ax)))* (180.0 / M_PI) ); // [-pigreco, pigreco] ---> [0, 2pigreco]
		pitch	=	(double)( (atan2(Ax, sqrt(Az*Az+Ay*Ay)))* (-180.0 / M_PI) );
		//yaw		= 	(double)( (atan2(Az, sqrt(Ax*Ax+Ay*Ay)))* (-180.0 / M_PI) );
	}
}


void MPU6050_Read_Gyro (I2C_HandleTypeDef* hi2c1){
	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data_Gyro, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data_Gyro[0] << 8 | Rec_Data_Gyro [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data_Gyro[2] << 8 | Rec_Data_Gyro [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data_Gyro[4] << 8 | Rec_Data_Gyro [5]);

	/*** convert the RAW values into dps (�/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

	if(MPU6050_Calib_status == 1){
		Gx = Gx - Gx_0;
		Gy = Gy - Gy_0;
		Gz = Gz - Gz_0;

		// angoli ottenuti solo dal giroscopio
		Gx_angle += (Gx * dTime/1000); // the gyro drifts over time
		Gy_angle += (Gy * dTime/1000); // the gyro drifts over time
		Gz_angle += (Gz * dTime/1000); // the gyro drifts over time
	}
}

void MPU6050_Calibration(I2C_HandleTypeDef* hi2c1){
	  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
		  MPU6050_Read_Accel(hi2c1);
		  MPU6050_Read_Gyro(hi2c1);
		  Ax_0 += Ax;
		  Ay_0 += Ay;
		  Az_0 += Az;

		  Gx_0 += Gx;
		  Gy_0 += Gy;
		  Gz_0 += Gz;

		  HAL_Delay(10);
	  }
	  Ax_0 = Ax_0/100;
	  Ay_0 = Ay_0/100;
	  Az_0 = Az_0/100;

	  Gx_0 = Gx_0/100;
	  Gy_0 = Gy_0/100;
	  Gz_0 = Gz_0/100;

	  MPU6050_Calib_status = 1;
}



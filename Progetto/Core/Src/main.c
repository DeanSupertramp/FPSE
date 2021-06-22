/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "myLib/PID.h"
#include "myLib/StepperMotor.h"
#include "math.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float pid_error, vel;
//#define steps_per_rev_motor 200
//#define microsteps_set 8
//int steps_per_rev_eff = (steps_per_rev_motor * microsteps_set * 2); // 2 perchè il periodo generato si ottiene da 2 toggle dovuti agli interrupt

//#define clk_freq 84000000
//#define prescaler 83+1
//#define counter 99+1
//#define freq_pulse (clk_freq)/((prescaler)*(counter))
//#define ms_per_pulse (2/freq_pulse)
//#define R_wheel 0.0325 // metri
//#define L_body 0.14 // metri

int clk_freq = 84000000;
int prescaler = 83+1;
int counter = 99+1;
float freq_pulse; //(clk_freq)/((prescaler)*(counter))
float s_per_pulse; // (1/freq_pulse)

//float angular_speed = steps_per_rev_eff*ms_per_pulse;
float angular_speed; // = steps_per_rev_eff*ms_per_pulse;
//float rel_speed;
int i, j, k;
//float seconds;
int j_state, k_state = 0;
uint32_t last = 0;
//float VELOCITY = 0;
//uint8_t direction;

#define STOP 	'S'
#define GO 		'G'
#define BACK 	'B'
#define LEFT 	'L'
#define RIGHT 	'R'
#define VEL		'V'
#define TEST	'T'
#define IDLE	 0

//unsigned char readBuf[1];
uint8_t read_value;
volatile ITStatus UartReady = SET; // Uart READY Flag



/* Controller parameters */
#define PID_KP  10.0f
#define PID_KI  200.0f
#define PID_KD 	 0.0f

#define PID_TAU 0.0f	/* A larger value of tau means the signal is filtered more heavily.
							As tau approaches zero, the differentiator approaches a 'pure differentiator'
							with no filtering. */

//#define PID_LIM_MIN (-281.25f)
//#define PID_LIM_MAX  (281.25f)

#define PID_LIM_MIN 		(-375.0f)
#define PID_LIM_MAX  		(375.0f)

#define PID_LIM_MIN_INT		(PID_LIM_MIN)
#define PID_LIM_MAX_INT  	(PID_LIM_MAX)

#define SAMPLE_TIME_S 		0.01f

float freq, dTime;

float pitch, roll, yaw;
float angleX = 0.0f;
float angleY = 0.0f;

uint8_t MPU6050_Calib_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void stopMotor(void);
void read_UART_input(void);

void USART2_IRQHandler(void){
	HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	// Set transmission flag: transfer complete
	UartReady = SET;
}


void TIM2_IRQHandler(void) {
	j++;
	HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void) {
	k++;
	HAL_TIM_IRQHandler(&htim3);
}

void TIM6_DAC_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  // Check which version of the timer triggered this callback
 // if (htim == &htim6 ){
	if(htim->Instance == TIM6){
		stopMotor(&htim2,&htim3, &htim6);
		HAL_TIM_Base_Stop_IT(&htim6);
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
		__HAL_TIM_CLEAR_FLAG(&htim6, TIM_IT_UPDATE);
  }
}

//void setDirection(uint8_t dir){
//	if (dir == 0){
//		HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
//	}else{
//		HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_RESET);
//	}
//}
//
//void getDirection(void){
//	if(VELOCITY>0){
//		direction = 0;
//	}else{
//		direction = 1;
//	}
//}
//
//void setSpeed(float speed){ // m/s
////	counter = (clk_freq)/((prescaler)*(freq_pulse));
////	freq_pulse = (1/s_per_pulse);
////	s_per_rev =
////	s_per_pulse = s_per_rev/steps_per_rev_eff;
////	0.32 = s_per_pulse * steps_per_rev_eff;
////	angular_speed = 2*M_PI/0.32;
//
//
//
//	rel_speed = (float)(19.635 * R_wheel);
//	float x = (float)(rel_speed/speed);	// 0.6381375
////	float x = 0.6381375/speed;
//	TIM2->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
//	TIM3->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
//}
//
//void setSpeedPID(float speedPID){ // deg/s
////	counter = (clk_freq)/((prescaler)*(freq_pulse));
////	freq_pulse = (1/s_per_pulse);
////	s_per_rev =
////	s_per_pulse = s_per_rev/steps_per_rev_eff;
////	0.32 = s_per_pulse * steps_per_rev_eff;
////	angular_speed = 2*M_PI/0.32;
//
//	if(speedPID < 0.0){
//		speedPID = speedPID * (-1.0f);
//	}
//
//	float x = (float)( (360.0f)/(0.32*speedPID) );	// deg/sec
//
//	//float x = (float)( (0.32*speedPID/(2*M_PI)) );	// deg/sec
//	TIM2->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
//	TIM3->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
//}
//
//void simpleGO(void){
//	//HAL_TIM_Base_Start_IT(&htim6);
//
//	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
//
//	if(HAL_TIM_OC_GetState(&htim2) != HAL_TIM_STATE_BUSY){
//		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
//	}
//
//	if(HAL_TIM_OC_GetState(&htim3) != HAL_TIM_STATE_BUSY){
//		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
//	}
//	readBuf[0]=0;
//}
//
//void goForward(float metri){
//	HAL_TIM_Base_Start_IT(&htim6);
//	float seconds = metri/VELOCITY;
//	TIM6->ARR = (uint32_t)(10000*seconds);
//
//	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
//	readBuf[0]=0;
//}
//
//void goBackward(float metri){
//	HAL_TIM_Base_Start_IT(&htim6);
//	float seconds = metri/VELOCITY;
//	TIM6->ARR = (uint32_t)(10000*seconds);
//
//	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_RESET);
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
//	readBuf[0]=0;
//}
//
//void rotateL(float rad){
//	HAL_TIM_Base_Start_IT(&htim6);
////	float metri = rad/L_body;
//	float 	seconds = (rad*L_body)/VELOCITY;
//	TIM6->ARR = (uint32_t)(10000*seconds);
//
//	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
//	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
//
//	readBuf[0]=0;
//}
//
//void rotateR(float rad){
//	HAL_TIM_Base_Start_IT(&htim6);
////	float metri = rad/L_body;
//	float seconds = (rad*L_body)/VELOCITY;
//	TIM6->ARR = (uint32_t)(10000*seconds);
//	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
//	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
//	readBuf[0]=0;
//
//}
//
//void stopMotor(void){
//	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
//	HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
//	HAL_TIM_Base_Stop_IT(&htim6);
//	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
//	__HAL_TIM_CLEAR_FLAG(&htim6, TIM_IT_UPDATE);
//	readBuf[0]=0;
//	i=0; // escludo eventuali overflow
//	j=0;
//}
//
//
#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

double Ax, Ay, Az, Gx, Gy, Gz;
double Ax_angle, Ay_angle, Az_angle, Gx_angle, Gy_angle, Gz_angle;
double Ax_0, Ay_0, Az_0, Gx_0, Gy_0, Gz_0;


uint8_t check;
uint8_t Data;
void MPU6050_Init (void){
	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &check, 1, 1000);
	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void MPU6050_getStatus(void){
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &check, 1, 1000);
	if (check != 104){
		MPU6050_Init();
	}
}


uint8_t Rec_Data_Accel[6];
void MPU6050_Read_Accel (void){
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data_Accel, 6, 1000);

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

uint8_t Rec_Data_Gyro[6];

void MPU6050_Read_Gyro (void){
	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data_Gyro, 6, 1000);

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

void MPU6050_Calibration(void){
	  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
		  MPU6050_Read_Accel();
		  MPU6050_Read_Gyro();
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

void complementaryFilter(void){
	angleX = (0.97 * (angleX+Gy*(dTime/1000))) + 0.03*pitch;
	angleY = (0.97 * (angleY+Gx*(dTime/1000))) + 0.03*roll;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	steps_per_rev_eff = (steps_per_rev_motor * microsteps_set * 2);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  arm_pid_instance_f32 PID;

  /* Set PID parameters */
   /* Set this for your needs */
   PID.Kp = PID_KP;        /* Proporcional */
   PID.Ki = PID_KI;        /* Integral */
   PID.Kd = PID_KD;        /* Derivative */

   /* Initialize PID system, float32_t format */
   arm_pid_init_f32(&PID, 1);



  /* Init PID controller */
  PIDController pid = { PID_KP, PID_KI, PID_KD,
                        PID_TAU,
                        PID_LIM_MIN, PID_LIM_MAX,
						PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                        SAMPLE_TIME_S };

  PIDController_Init(&pid);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);

  NVIC_SetPriority(TIM3_IRQn, 0);
  NVIC_EnableIRQ(TIM3_IRQn);

  NVIC_SetPriority(TIM6_DAC_IRQn, 0);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  __enable_irq();

//  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Stop_IT(&htim6);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_IT_UPDATE);


  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);

  MPU6050_Init();
  MPU6050_Calibration();

  float setpoint = 0.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	dTime = HAL_GetTick() - last;
	last = HAL_GetTick();
//	freq = HAL_GetTickFreq();

	MPU6050_getStatus();
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();

	read_UART_input();

//	VELOCITY = 0.6381/4;
//	setSpeed(VELOCITY);
	VELOCITY = pid.out;
	setSpeedPID(VELOCITY);

	getDirection();
	setDirection(direction);

	if ((readBuf[0] == STOP) ){
		stopMotor(&htim2,&htim3, &htim6);
	}
	if ((readBuf[0] == GO) ){
		goForward(&htim2, &htim3, &htim6, 0.1);
	}
	if ((readBuf[0] == BACK) ){
		goBackward(&htim2, &htim3, &htim6, 0.2);
	}
	if ((readBuf[0] == LEFT) ){
		rotateL(&htim2, &htim3, &htim6, M_PI/2);
	}
	if ((readBuf[0] == RIGHT) ){
		rotateR(&htim2, &htim3, &htim6, M_PI/2);
	}

	if((readBuf[0] == TEST) ){
		simpleGO(&htim2, &htim3);
	}

	complementaryFilter();

    PIDController_Update(&pid, setpoint, angleX);	//IN: angle in deg; OUT: angular speed (deg/sec)

    pid_error = pid.prevError;
    vel = arm_pid_f32(&PID, pid_error);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  // __HAL_TIM_CLEAR_FLAG(&htim6, TIM_SR_UIF);//Add this statement to solve the problem

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin|LD2_Pin|DIR_SX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_DX_Pin DIR_SX_Pin */
  GPIO_InitStruct.Pin = DIR_DX_Pin|DIR_SX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void read_UART_input() {
	if(UartReady == SET) {
		UartReady = RESET;
		HAL_UART_Receive_IT(&huart2, (uint8_t*)readBuf, 1);
		//read_value = atoi(readBuf);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

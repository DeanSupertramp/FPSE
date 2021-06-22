#include "myLib/StepperMotor.h"

steps_per_rev_eff = (int)(steps_per_rev_motor * microsteps_set * 2);


void setDirection(uint8_t dir){
	if (dir == 0){
		HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_RESET);
	}
}

void getDirection(void){
	if(VELOCITY>0){
		direction = 0;
	}else{
		direction = 1;
	}
}

void setSpeed(float speed){ // m/s
//	counter = (clk_freq)/((prescaler)*(freq_pulse));
//	freq_pulse = (1/s_per_pulse);
//	s_per_rev =
//	s_per_pulse = s_per_rev/steps_per_rev_eff;
//	0.32 = s_per_pulse * steps_per_rev_eff;
//	angular_speed = 2*M_PI/0.32;



	rel_speed = (float)(19.635 * R_wheel);
	float x = (float)(rel_speed/speed);	// 0.6381375
//	float x = 0.6381375/speed;
	TIM2->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
	TIM3->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
}

void setSpeedPID(float speedPID){ // deg/s
//	counter = (clk_freq)/((prescaler)*(freq_pulse));
//	freq_pulse = (1/s_per_pulse);
//	s_per_rev =
//	s_per_pulse = s_per_rev/steps_per_rev_eff;
//	0.32 = s_per_pulse * steps_per_rev_eff;
//	angular_speed = 2*M_PI/0.32;

	if(speedPID < 0.0){
		speedPID = speedPID * (-1.0f);
	}

	float x = (float)( (360.0f)/(0.32*speedPID) );	// deg/sec

	//float x = (float)( (0.32*speedPID/(2*M_PI)) );	// deg/sec
	TIM2->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
	TIM3->ARR = (uint32_t)((100-1)*x);	// (100-1)*16
}

void simpleGO(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3){
	//HAL_TIM_Base_Start_IT(&htim6);

	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);

	if(HAL_TIM_OC_GetState(htim2) != HAL_TIM_STATE_BUSY){
		HAL_TIM_OC_Start_IT(htim2, TIM_CHANNEL_1);
	}

	if(HAL_TIM_OC_GetState(htim3) != HAL_TIM_STATE_BUSY){
		HAL_TIM_OC_Start_IT(htim3, TIM_CHANNEL_1);
	}
	readBuf[0]=0;
}

void goForward(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float metri){
	HAL_TIM_Base_Start_IT(htim6);
	float seconds = metri/VELOCITY;
	TIM6->ARR = (uint32_t)(10000/seconds);

	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
	HAL_TIM_OC_Start_IT(htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(htim3, TIM_CHANNEL_1);
	readBuf[0]=0;
}

void goBackward(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float metri){
	HAL_TIM_Base_Start_IT(htim6);
	float seconds = metri/VELOCITY;
	TIM6->ARR = (uint32_t)(10000/seconds);

	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_RESET);
	HAL_TIM_OC_Start_IT(htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(htim3, TIM_CHANNEL_1);
	readBuf[0]=0;
}

void rotateL(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float rad){
	HAL_TIM_Base_Start_IT(htim6);
//	float metri = rad/L_body;
	float 	seconds = (rad*L_body)/VELOCITY;
	TIM6->ARR = (uint32_t)(10000*seconds);

	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
	HAL_TIM_OC_Start_IT(htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(htim3, TIM_CHANNEL_1);

	readBuf[0]=0;
}

void rotateR(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float rad){
	HAL_TIM_Base_Start_IT(htim6);
//	float metri = rad/L_body;
	float seconds = (rad*L_body)/VELOCITY;
	TIM6->ARR = (uint32_t)(10000*seconds);
	HAL_GPIO_WritePin(GPIOA, DIR_DX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, DIR_SX_Pin, GPIO_PIN_SET);
	HAL_TIM_OC_Stop_IT(htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(htim3, TIM_CHANNEL_1);
	readBuf[0]=0;

}

void stopMotor(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6){
	HAL_TIM_OC_Stop_IT(htim2, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Stop_IT(htim6);
	__HAL_TIM_CLEAR_FLAG(htim6, TIM_SR_UIF);
	__HAL_TIM_CLEAR_FLAG(htim6, TIM_IT_UPDATE);
	readBuf[0]=0;
//	i=0; // escludo eventuali overflow
//	j=0;
}




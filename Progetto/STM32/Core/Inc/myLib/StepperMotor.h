#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <main.h>

#define steps_per_rev_motor 200
#define microsteps_set 8
#define L_body 0.14 // metri
#define R_wheel 0.0325 // metri
int steps_per_rev_eff; // 2 perchè il periodo generato si ottiene da 2 toggle dovuti agli interrupt
uint8_t direction;
float rel_speed;

unsigned char readBuf[1];

void setDirection(uint8_t dir);
void getDirection(float VELOCITY);
void setSpeed(float speed);
void setSpeedPID(float speedPID);
void simpleGO(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3);
void goForward(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float VELOCITY, float metri);
void goBackward(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float VELOCITY, float metri);
void rotateL(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float VELOCITY, float rad);
void rotateR(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float VELOCITY, float rad);
void stopMotor(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6);

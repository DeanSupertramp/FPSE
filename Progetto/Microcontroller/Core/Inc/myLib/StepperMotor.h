#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <main.h>


#define steps_per_rev_motor 200
#define microsteps_set 8
int steps_per_rev_eff; // 2 perchè il periodo generato si ottiene da 2 toggle dovuti agli interrupt
float VELOCITY;
uint8_t direction;
float rel_speed;
#define R_wheel 0.0325 // metri
#define L_body 0.14 // metri

unsigned char readBuf[1];

void setDirection(uint8_t dir);
void getDirection(void);
void setSpeed(float speed);
void setSpeedPID(float speedPID);
void simpleGO(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3);
void goForward(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float metri);
void goBackward(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float metri);
void rotateL(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float rad);
void rotateR(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6, float rad);
void stopMotor(TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim3, TIM_HandleTypeDef* htim6);

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define Vref 3.3
#define Avg_Slope .0025
#define V25 0.76

void Error_Handler(void);

void ADC_config(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time);

float getADCraw(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time);

float getLM35(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time);

float getTMP36(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time);

float getInternalTemp(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time);

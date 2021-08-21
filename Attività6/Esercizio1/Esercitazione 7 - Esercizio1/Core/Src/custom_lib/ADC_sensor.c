#include "custom_lib/ADC_sensor.h"

float getADCraw(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time){	// Acquire LM35 temperature value
	uint16_t rawValue;
	ADC_config(hadc, sConfig, channel, rank, sampling_time);
	// Start ADC
	HAL_ADC_Start(hadc);
	// Polling mode
	if(HAL_ADC_PollForConversion(hadc, 1000) == HAL_OK){
	// Read the analog input value
		rawValue = HAL_ADC_GetValue(hadc);
	}
	HAL_ADC_Stop(hadc);
	return rawValue;
}

float getLM35(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time){
	float LM35_raw = getADCraw(hadc, sConfig, channel, rank, sampling_time);
	return ((float)LM35_raw)* Vref/(4095*0.01) ; //		LM35 0.08= 3.3/(4095*10mv)	=0.08		(10mV: risoluzione mV/grado)
}

float getTMP36(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time){
	float TMP36_raw = getADCraw(hadc, sConfig, channel, rank, sampling_time);
	return ((float)TMP36_raw*(Vref/4095)-0.5)*100;
}

float getInternalTemp(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time){
	float InternalTemp_raw = getADCraw(hadc, sConfig, channel, rank, sampling_time);
	return (float)((Vref*InternalTemp_raw/4095-V25)/Avg_Slope)+25;;
}

void ADC_config(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig, uint32_t channel, uint32_t rank, uint32_t sampling_time){
	sConfig->Channel = channel;
	sConfig->Rank = rank;
	sConfig->SamplingTime = sampling_time;
	if (HAL_ADC_ConfigChannel(hadc, sConfig) != HAL_OK)	{
		Error_Handler();
	}
}

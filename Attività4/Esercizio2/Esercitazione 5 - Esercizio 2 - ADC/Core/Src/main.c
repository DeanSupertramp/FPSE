/* USER CODE BEGIN Header */
#include "stdbool.h"
#include "custom_lib/avg_lib.h"
#include "custom_lib/ADC_sensor.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Variables
bool alarm = true;

uint16_t raw_in=0;
float TMP36 = 0.0;
float LM35=0.0;
float media = 0.0;
uint16_t var = 0;
float val = 0.0;

float pot = 0.0;
float internalTemp = 0.0;
float returnADC = 0.0;


uint8_t valByte;
uint32_t lastValue_ms;
uint32_t currentValue_ms;

ADC_ChannelConfTypeDef sConfig = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


float sample_pot[10];
float sample_LM35[10];
float sample_internalTemp[10];
float sample_TMP36[10];
float sample_returnADC[10];

float arrNumbers_pot[5] = {0};
float arrNumbers_LM35[5] = {0};
float arrNumbers_internalTemp[5] = {0};
float arrNumbers_TMP36[5] = {0};
float arrNumbers_returnADC[5] = {0};

// pot
int pos_pot = 0;
float newAvg_pot = 0.0;float sum_pot = 0;
int len_pot = sizeof(arrNumbers_pot) / sizeof(int);
int count_pot = sizeof(sample_pot) / sizeof(int);
int i_pot = 0;

// LM35
int pos_LM35 = 0;
float newAvg_LM35 = 0.0;
float sum_LM35 = 0;
int len_LM35 = sizeof(arrNumbers_LM35) / sizeof(int);
int count_LM35 = sizeof(sample_LM35) / sizeof(int);
int i_LM35 = 0;

// internalTemp
int pos_internalTemp = 0;
float newAvg_internalTemp = 0.0;
float sum_internalTemp = 0.0;
int len_internalTemp = sizeof(arrNumbers_internalTemp) / sizeof(int);
int count_internalTemp = sizeof(sample_internalTemp) / sizeof(int);
int i_internalTemp = 0;

// TMP36
int pos_TMP36 = 0;
float newAvg_TMP36 = 0.0;
float sum_TMP36 = 0.0;
int len_TMP36 = sizeof(arrNumbers_TMP36) / sizeof(int);
int count_TMP36 = sizeof(sample_TMP36) / sizeof(int);
int i_TMP36 = 0;

// returnADC
int pos_returnADC = 0;
float newAvg_returnADC = 0;
float sum_returnADC = 0;
int len_returnADC = sizeof(arrNumbers_returnADC) / sizeof(int);
int count_returnADC= sizeof(sample_returnADC) / sizeof(int);
int i_returnADC = 0;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  alarm = false;
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, valByte);
  HAL_GPIO_WritePin(GPIOA, LED_Esterno_Pin, GPIO_PIN_RESET);

  lastValue_ms = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  // ------------------ returnADC ------------------------
	  returnADC = getADCraw(&hadc2, &sConfig, ADC_CHANNEL_6, 1, ADC_SAMPLETIME_28CYCLES);
	  sample_returnADC[i_returnADC] = returnADC;
	  newAvg_returnADC = movingAvg(arrNumbers_returnADC, &sum_returnADC, pos_returnADC, len_returnADC, sample_returnADC[i_returnADC]);
	  updateAvg(&pos_returnADC, len_returnADC, &i_returnADC, count_returnADC);

	  if(alarm == false){
		  // ------------------ LM35 ------------------------
		  LM35 = getLM35(&hadc1, &sConfig, ADC_CHANNEL_10, 1, ADC_SAMPLETIME_84CYCLES);
		  sample_LM35[i_LM35] = LM35;
		  newAvg_LM35 = movingAvg(arrNumbers_LM35, &sum_LM35, pos_LM35, len_LM35, sample_LM35[i_LM35]);
		  updateAvg(&pos_LM35, len_LM35, &i_LM35, count_LM35);

		  // ------------------ INTERNAL SENS ------------------------
		  internalTemp = getInternalTemp(&hadc1, &sConfig, ADC_CHANNEL_TEMPSENSOR, 1, ADC_SAMPLETIME_56CYCLES);
		  sample_internalTemp[i_internalTemp] = internalTemp;
		  newAvg_internalTemp = movingAvg(arrNumbers_internalTemp, &sum_internalTemp, pos_internalTemp, len_internalTemp, sample_internalTemp[i_internalTemp]);
		  updateAvg(&pos_internalTemp, len_internalTemp, &i_internalTemp, count_internalTemp);

		  // ------------------ TMP36 ------------------------
		  TMP36 = getTMP36(&hadc1, &sConfig, ADC_CHANNEL_1, 1, ADC_SAMPLETIME_56CYCLES);
		  sample_TMP36[i_TMP36] = TMP36;
		  newAvg_TMP36 = movingAvg(arrNumbers_TMP36, &sum_TMP36, pos_TMP36, len_TMP36, sample_TMP36[i_TMP36]);
		  updateAvg(&pos_TMP36, len_TMP36, &i_TMP36, count_TMP36);

		  if( (newAvg_LM35 >030.0) | (newAvg_internalTemp > 28.0) | (newAvg_TMP36 >030.0) ){
			  alarm = true;
			  lastValue_ms = HAL_GetTick();
		  }else{
			  __NOP();
		  }
	  }else{

		  if(HAL_GetTick() - lastValue_ms < 10000){
			  pot = getADCraw(&hadc1, &sConfig, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_28CYCLES);
			  sample_pot[i_pot] = pot;
			  newAvg_pot = movingAvg(arrNumbers_pot, &sum_pot, pos_pot, len_pot, sample_pot[i_pot]);
			  updateAvg(&pos_pot, len_pot, &i_pot, count_pot);

			  valByte = (uint8_t)(newAvg_pot/16.0); // valore a 12 bit scalato a 8 bit
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, valByte);
			  HAL_GPIO_WritePin(GPIOA, LED_Esterno_Pin, GPIO_PIN_SET);
		  }else{
			  alarm = false;
			  valByte = (uint8_t)(0.0); // mettere 0
			  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, valByte);
			  HAL_GPIO_WritePin(GPIOA, LED_Esterno_Pin, GPIO_PIN_RESET);
		  }
	  }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */
//
// ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  *///  sConfig.Channel = ADC_CHANNEL_1;//  sConfig.Rank = 1;//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)//  {//    Error_Handler();//  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */
  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */
  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */
  /* USER CODE END DAC_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_Esterno_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Esterno_Pin */
  GPIO_InitStruct.Pin = LED_Esterno_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Esterno_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

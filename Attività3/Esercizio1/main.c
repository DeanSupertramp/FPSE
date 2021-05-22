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

/* USER CODE BEGIN PV */
volatile int button1_pressed_flag = 0;
volatile int button2_pressed_flag = 0;
volatile int button3_pressed_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void EXTI15_10_IRQHandler(void){
	if((EXTI->PR >> 10) & 0x01){ // equivalente a if(EXTI->PR & (0x01 << 10))
		//Gestione PC_10
		NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
		button1_pressed_flag = 1;
		EXTI->PR |= (0x01 >> 10); // Clear del pending request register (sto elaborando l'interrupt) SEMPRE ALLA FINE
	}
	else if((EXTI->PR >> 11) & 0x01){ // equivalente a if(EXTI->PR & (0x01 << 10))
	//else if(EXTI->PR & (0x01 << 11)){
		//Gestione PC_11
		NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
		button2_pressed_flag = 1;
		EXTI->PR |= (0x01 >> 11); // Clear del pending request register (sto elaborando l'interrupt) SEMPRE ALLA FINE
	}
}

void EXTI2_IRQHandler(void){
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	button3_pressed_flag = 1;
	EXTI->PR |= (0x01 >> 2); // Clear del pending request register (sto elaborando l'interrupt) SEMPRE ALLA FINE
}

void processA(void);
void processB(void);
void processC(void);

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
  /* USER CODE BEGIN 2 */

	RCC->AHB1ENR |= (0x01 << 2) | (0x01 <<0); // abilito pin della porta C e porta A. Eq. a 5 (0101)

	// -------------------- INPUT CONFIG -------------------------
	// Attivo PC_2 (Esterno)
	GPIOC->MODER &= ~(0x03 << 4); //clear dei due bit della porta 2
	GPIOC->MODER |= (0x00 << 4); //set input
	GPIOC->PUPDR &= ~(0x03 << 4); //clear
	GPIOC->PUPDR |= (0x01 << 4); //set pullup

	// Attivo PC_10 (Esterno)
	GPIOC->MODER &= ~(0x03 << 20); //clear dei due bit della porta 10
	GPIOC->MODER |= (0x00 << 20); //set input
	GPIOC->PUPDR &= ~(0x03 << 20); //clear
	GPIOC->PUPDR |= (0x01 << 20); //set pullup

	// Attivo PC_11 (Esterno)
	GPIOC->MODER &= ~(0x03 << 22); //clear dei due bit della porta 11
	GPIOC->MODER |= (0x00 << 22); //set input
	GPIOC->PUPDR &= ~(0x03 << 22); //clear
	GPIOC->PUPDR |= (0x01 << 22); //set pullup

	// -------------------- OUTPUT CONFIG -------------------------
	// Attivo PA_5
	GPIOA->MODER &= ~(0x03 << 10); //clear
	GPIOA->MODER |= (0x01 << 10); //set output
	GPIOA->PUPDR &= ~(0x03 << 10); //clear
	GPIOA->PUPDR |= (0x01 << 10); //set pullup
	GPIOA->OTYPER &= ~(0x01 << 5); //set push-pull
	// Set alto all'avvio
	GPIOA->ODR |= (0x01 << 5);

	// Attivo PA_10
	GPIOA->MODER &= ~(0x03 << 20); //clear
	GPIOA->MODER |= (0x01 << 20); //set output
	GPIOA->PUPDR &= ~(0x03 << 20); //clear
	GPIOA->PUPDR |= (0x01 << 20); //set pullup
	GPIOA->OTYPER &= ~(0x01 << 10); //set push-pull
	// Set alto all'avvio
	GPIOA->ODR |= (0x01 << 10);

	// Attivo PA_15
	GPIOA->MODER &= ~(0x03 << 30); //clear
	GPIOA->MODER |= (0x01 << 30); //set output
	GPIOA->PUPDR &= ~(0x03 << 30); //clear
	GPIOA->PUPDR |= (0x01 << 30); //set pullup
	GPIOA->OTYPER &= ~(0x01 << 15); //set push-pull
	// Set alto all'avvio
	GPIOA->ODR |= (0x01 << 15);

	// --------------------- GESTIONE INTERRUPT -------------------
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //equivalente a (0x01 << 14). Abilito il clock per il componente di interrupt su quella porta
	// PC_10
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PC; //abilito l'ingresso PC_14 come interrupt. Eq. a (0x200	0000 0010 0000 0000	512)
	EXTI->IMR |= (0x01 << 10); // set interrupt non maskable sulla porta 14
	EXTI->RTSR |= (0x01 << 10); //set fronte di salita
	// PC_11
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PC; //abilito l'ingresso PC_15 come interrupt. Eq. a (0x2000	0010 0000 0000 0000 - 8192)
	EXTI->IMR |= (0x01 << 11); // set interrupt non maskable sulla porta 15
	EXTI->RTSR |= (0x01 << 11); //set fronte di salita
	// PC_2
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC; //abilito l'ingresso PC_2 come interrupt. Eq. a (0x200	0000 0010 0000 0000	512)
	EXTI->IMR |= (0x01 << 2); // set interrupt non maskable sulla porta 2
	EXTI->RTSR |= (0x01 << 2); //set fronte di salita

	// ABILITO INTERRUPT
	// in assembly
	//__asm volatile ("cpsie i" : : : "memory");
	__enable_irq();

	// CONFIGURAZIONE NVIC
	// PC_13
	NVIC_SetPriority(EXTI15_10_IRQn, 0); //abilito interrupt per tutte le linee da 10 a 15 con priorità '0' (la più alta)
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn); //clear di qualsiasi interrupt precedente l'avvio
	NVIC_EnableIRQ(EXTI15_10_IRQn); //avvio la richiesta interrupt

	// PC_2
	NVIC_SetPriority(EXTI2_IRQn, 0); //abilito interrupt per la linea 2 con priorità '0' (la più alta)
	NVIC_ClearPendingIRQ(EXTI2_IRQn); //clear di qualsiasi pending request precedente l'avvio
	NVIC_EnableIRQ(EXTI2_IRQn); //avvio la richiesta interrupt

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  if(button1_pressed_flag == 1){
		  processA();
	  }
	  else{
		  __NOP();
	  }

	  if(button2_pressed_flag == 1){
		  processB();
	  }
	  else{
		  __NOP();
	  }

	  if(button3_pressed_flag == 1){
		  processC();
	  }
	  else{
		  __NOP();
	  }

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void processA(void){ // Cambio stato (toggle) di tutti e tre i LED;
	GPIOA -> ODR ^= (0x01 << 5); // toggle su PA_5
	GPIOA -> ODR ^= (0x01 << 10); // toggle su PA_10
	GPIOA -> ODR ^= (0x01 << 15); // toggle su PA_15
	HAL_Delay(100);
	//reset status flag
	button1_pressed_flag = 0;
}

void processB(void){ // Lampeggiamento intermittente dei tre LED, per almeno un secondo;
	for (int i=0; i<10; i++){
		GPIOA -> ODR ^= (0x01 << 5); // toggle su PA_5
		GPIOA -> ODR ^= (0x01 << 10); // toggle su PA_10
		GPIOA -> ODR ^= (0x01 << 15); // toggle su PA_15
		HAL_Delay(100);
	}
	//reset status flag
	button2_pressed_flag = 0;
}

void processC(void){ // Accensione e successivo spegnimento in sequenza dei tre LED;
	GPIOA -> ODR = (0x01 << 5); // set su PA_5
	HAL_Delay(250);
	GPIOA -> ODR = (0x01 << 10) | (0x01 << 5); // set su PA_10
	HAL_Delay(250);
	GPIOA -> ODR = (0x01 << 15) | (0x01 << 10) | (0x01 << 5); // set su PA_15
	HAL_Delay(250);
	GPIOA -> ODR = (0x01 << 15) | (0x01 << 10) | (0x00 << 5); // clear su PA_5
	HAL_Delay(250);
	GPIOA -> ODR = (0x01 << 15) | (0x00 << 10); // clear su PA_10
	HAL_Delay(250);
	GPIOA -> ODR = (0x00 << 15); // clear su PA_15
	//reset status flag
	button3_pressed_flag = 0;
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint32_t speed = 0;

volatile uint32_t SP_UP_lastSt = 0;
volatile uint32_t SP_UP_curSt = 0;
volatile uint32_t SP_UP_isPressed = 0;

volatile uint32_t SP_DOWN_lastSt = 0;
volatile uint32_t SP_DOWN_curSt = 0;
volatile uint32_t SP_DOWN_isPressed = 0;

volatile uint32_t BZ_lastSt = 0;
volatile uint32_t BZ_curSt = 0;
volatile uint32_t BZ_isPressed = 0;

int __io_putchar(int ch){
	ITM_SendChar(ch);
	return ch;
}

void beep(uint32_t count, uint32_t ms) { // function for zummer signal
	for (int f = 0; f < count; f++) {
		GPIOA->ODR |= GPIO_ODR_ODR15;
		HAL_Delay(ms);//delay_ms(ms);
		GPIOA->ODR &= ~GPIO_ODR_ODR15;
		HAL_Delay(ms);//delay_ms(ms);

	}
}

void displaySpeed(uint32_t speed) { // function for display speed
	switch (speed) {
	case 0:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR &= ~GPIO_ODR_ODR1;
		GPIOB->ODR &= ~GPIO_ODR_ODR2;
		GPIOB->ODR &= ~GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 1:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR &= ~GPIO_ODR_ODR2;
		GPIOB->ODR &= ~GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 2:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR |= GPIO_ODR_ODR2;
		GPIOB->ODR &= ~GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 3:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR |= GPIO_ODR_ODR2;
		GPIOB->ODR |= GPIO_ODR_ODR4;
		GPIOB->ODR &= ~GPIO_ODR_ODR5;
		break;
	case 4:
		GPIOB->ODR |= GPIO_ODR_ODR0;
		GPIOB->ODR |= GPIO_ODR_ODR1;
		GPIOB->ODR |= GPIO_ODR_ODR2;
		GPIOB->ODR |= GPIO_ODR_ODR4;
		GPIOB->ODR |= GPIO_ODR_ODR5;
		break;
	}
}

uint32_t ADC_Read_VBAT(ADC_HandleTypeDef* hadc) {
//	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
//	// select 4 channel
//	ADC1->SQR1 = 0; // 1 regular channel
//	ADC1->SQR2 = 0x00000000;
//	ADC1->SQR3 = 0x00000100; // 1 convert - channel 4
//	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
//	ADC1->CR2 |= ADC_CR2_SWSTART; // start adc convert
//	while (!(ADC1->SR & ADC_SR_EOC)) {
//	}; //wait for end of convert
	 HAL_ADCEx_InjectedStart(hadc);
	 HAL_ADCEx_InjectedPollForConversion(hadc, 10);
	 uint32_t vBat = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

	return vBat;  //ADC1->DR; // read adc data
}

uint32_t ADC_Read_JoystickA(ADC_HandleTypeDef* hadc) {
//	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
//	// select 0 channel
//	ADC1->SQR1 = 0; // 1 regular channel
//	ADC1->SQR2 = 0x00000000;
//	ADC1->SQR3 = 0x00000000; // 1 convert - channel 0
//	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
//	ADC1->CR2 |= ADC_CR2_SWSTART; // start adc convert
//	while (!(ADC1->SR & ADC_SR_EOC)) {
//	}; //wait for end of convert
	HAL_ADCEx_InjectedStart(hadc);
    HAL_ADCEx_InjectedPollForConversion(hadc, 10);
	uint32_t jA = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	return  jA; //ADC1->DR; // read adc data
}

uint32_t ADC_Read_JoystickB(ADC_HandleTypeDef* hadc) {
//	ADC1->CR2 &= ~(ADC_CR2_ADON); // turn of adc
//	// select 1 channel
//	ADC1->SQR1 = 0; // 1 regular channel
//	ADC1->SQR2 = 0x00000000;
//	ADC1->SQR3 = 0x00000001; // 1 convert - channel 1
//	ADC1->CR2 |= (ADC_CR2_ADON); // turn on adc
//	ADC1->CR2 |= ADC_CR2_SWSTART; // start adc convert
//	while (!(ADC1->SR & ADC_SR_EOC)) {
//	}; //wait for end of convert
	  HAL_ADCEx_InjectedStart(hadc);
	  HAL_ADCEx_InjectedPollForConversion(hadc, 10);
	  uint32_t jB = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	return jB; //ADC1->DR; // read adc data
}

void displayBat(uint32_t batVal) {
	if (batVal < 2270) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR &= ~GPIO_ODR_ODR7;
		GPIOB->ODR &= ~GPIO_ODR_ODR8;
		GPIOB->ODR &= ~GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 2388) && (batVal < 2490)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR &= ~GPIO_ODR_ODR8;
		GPIOB->ODR &= ~GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 2490) && (batVal < 2613)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR |= GPIO_ODR_ODR8;
		GPIOB->ODR &= ~GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 2613) && (batVal < 2728)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR |= GPIO_ODR_ODR8;
		GPIOB->ODR |= GPIO_ODR_ODR9;
		GPIOB->ODR &= ~GPIO_ODR_ODR10;
	} else if ((batVal >= 2728)) {
		GPIOB->ODR |= GPIO_ODR_ODR6;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		GPIOB->ODR |= GPIO_ODR_ODR8;
		GPIOB->ODR |= GPIO_ODR_ODR9;
		GPIOB->ODR |= GPIO_ODR_ODR10;
	}
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);

	// setup leds
	GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1
			| GPIO_CRL_CNF1 | GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
			GPIO_CRL_MODE4 | GPIO_CRL_CNF4 | GPIO_CRL_MODE5 | GPIO_CRL_CNF5
			| GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
			GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
	GPIOB->CRH &= ~( GPIO_CRH_MODE8 | GPIO_CRH_CNF8 | GPIO_CRH_MODE9
			| GPIO_CRH_CNF9 |
			GPIO_CRH_MODE10 | GPIO_CRH_CNF10); //reset leds

	GPIOB->CRL |= (GPIO_CRL_MODE0_0 | GPIO_CRL_MODE1_0 | GPIO_CRL_MODE2_0
			| GPIO_CRL_MODE4_0 | GPIO_CRL_MODE5_0 |
			GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0);
	GPIOB->CRH |= (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE10_0); //leds to output with max speed 10MHz


  beep(2, 500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  SP_UP_curSt = HAL_GPIO_ReadPin(SP_UP_GPIO_Port, SP_UP_Pin);
	  SP_DOWN_curSt = HAL_GPIO_ReadPin(SP_DOWN_GPIO_Port, SP_DOWN_Pin);
	  BZ_curSt = HAL_GPIO_ReadPin(BZ_GPIO_Port, BZ_Pin);
	  uint32_t A1 = ADC_Read_JoystickA(&hadc1);
	  uint32_t B1 = ADC_Read_JoystickB(&hadc1);
	  uint32_t vBat = ADC_Read_VBAT(&hadc1);

	  if ((SP_UP_curSt == 1)&&(SP_UP_lastSt == 0)){
		  SP_UP_isPressed = 1;
	  }else{
		  SP_UP_isPressed = 0;
	  }

	  if (SP_UP_isPressed == 1){
		  if(speed < 5){
			  speed++;
			  //printf("up, speed %li\n", speed);
		  }
	  }

	  if ((SP_DOWN_curSt == 1)&&(SP_DOWN_lastSt == 0)){
		  SP_DOWN_isPressed = 1;
	  }else{
		  SP_DOWN_isPressed = 0;
	  }

	  if (SP_DOWN_isPressed == 1){
		  if(speed > 0){
			  speed--;
			  //printf("down, speed %li\n", speed);
		  }
	  }

	  if ((BZ_curSt == 1)&&(BZ_lastSt == 0)){
		  BZ_isPressed = 1;
	  }else{
		  BZ_isPressed = 0;
	  }

	  if (BZ_isPressed == 1){
		  beep(1,1000);
		  //printf("beep! \n");
	  }
	  displaySpeed(speed);
	  displayBat(vBat);

	  printf("ControlPanel Test, btn_UP_isPressed: %li, btn_BZ_isPressed: %li, btn_DOWN_isPressed: %li, speed: %li, Joystick Alpha 1: %lu, Joystick Beta 1: %lu, vBat: %lu \n", SP_UP_isPressed, BZ_isPressed, SP_DOWN_isPressed, speed, A1, B1, vBat);
	  SP_UP_lastSt = SP_UP_curSt;
	  SP_DOWN_lastSt = SP_DOWN_curSt;
	  BZ_lastSt = BZ_curSt;
	  HAL_Delay(100);
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : SP_UP_Pin SP_DOWN_Pin */
  GPIO_InitStruct.Pin = SP_UP_Pin|SP_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BZ_Pin */
  GPIO_InitStruct.Pin = BZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BZ_GPIO_Port, &GPIO_InitStruct);

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

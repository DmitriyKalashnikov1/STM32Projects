/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdbool.h>
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "Drivers/myGPIO.h"
#include "Drivers/myVoltageRead.h"
#include "Drivers/ACS712.h"
#include "Drivers/myNRF.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct PID {
	float Kp;
	float Ki;
	float Kd;
} PID_t;

typedef enum PID_Param {
	Output_Current,
	Input_Power,
} PID_Param_e;

typedef struct MESSAGE {
	uint32_t id;
    uint32_t V_out_int;
    uint32_t I_out_int;

} message_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Конфигурационные параметры ****************************************************************/
#define ADDR_WIDTH 5 // Размер адреса принимающего канал приемопередатчика
#define PLD_S 8 // Размер пакета принимаемых данных, байт
#define FILTER_SIZE 10 // Объем накапливаемых значений зарядного тока в алгоритме фильтрации
/********************************************************************************************/
#define period_WAIT 0x66A0
#define period_CHRG 0x66C0


/* [A] */
#define MAX_CURRENT 75.0f // Макисмальный допустимый ток в нагрузке
#define MIN_CURRENT 1.0f  // Минимальный допустимый ток в нагрузке
#define TARGET_CURRENT 70.0f // Целевое значение тока в нагрузке (для LB было 5.0f)

/* [B] */
#define MAX_CHARGE 35.0f // Макисмальное допустимое напряжение на нагрузке
#define MIN_CHARGE 15.0f  // Минимальное допустимое напряжение на нагрузке

#define VOLTAGE_SCALE 0.0855f // Коэфициент преобразования в вольты
#define CURRENT_SCALE 0.05f // [мA/В] Коэффициент преобразования в амперы

// пределы срабатывания фильтра
#define FILTER_INT_CURRENT_THRESHOLD 200
#define FILTER_INT_VOLTAGE_THRESHOLD 200

#define PID_TYPE Output_Current
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

HRTIM_HandleTypeDef hhrtim1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Переменные для таймеров инвертера
uint16_t shift = 0;
uint16_t shift_normal = 0x0080;
uint16_t min_shift = 0x0080;

// Переменные для ПИД контроллера
float input = 0.0;  // PID constants
PID_t coeffs;
float previousError = 0.0f;   // Previous error for derivative term
float integral = 0.0f;        // Integral term
float P = 0.0f;
float I = 0.0f;
float D = 0.0f;
float error = 0.0f;
float ref = 5.0f;
float output = 0.0f;

// Переменные для обработки принятых пакетов данных


message_t message = {
	.id = 0,
	.V_out_int = 0,
	.I_out_int = 0
};

volatile uint32_t copter_id = 0;

// Физические измерения
volatile float copter_voltage = 0.0f;
volatile float copter_current = 0.0f;
volatile float copter_power = 0.0f;
float station_current = 0.0f;
float station_voltage = 0.0f;
float station_power = 0.0f;


// Переменные для фильтрации
volatile uint32_t receivedCurrents[FILTER_SIZE] = { 0 };
volatile uint32_t receivedVoltages[FILTER_SIZE] = { 0 };

volatile uint32_t sumI = 0;
volatile uint32_t cntI = 0;
volatile uint32_t sumV = 0;
volatile uint32_t cntV = 0;


// Переменные для коммуникации по SPI
const uint32_t CPOL = SPI_CR1_CPOL * 0;
const uint32_t CPHA = SPI_CR1_CPHA * 0;
const uint32_t pipe = 1024; //111156789; // адрес рабочей трубы;

// Переменные-счетчики
volatile uint32_t currentIndex = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_HRTIM1_Init(void);
/* USER CODE BEGIN PFP */
/*-------------- Функция итеративной фильтрации принятого значения зарядного тока --------------*/
void Filter()
{

	for (int i = 0; i < FILTER_SIZE; i++) {
		if (receivedCurrents[i] > FILTER_INT_CURRENT_THRESHOLD) {
			sumI += receivedCurrents[i];
			cntI++;
		}

		if (receivedVoltages[i] > FILTER_INT_VOLTAGE_THRESHOLD) {
			sumV += receivedVoltages[i];
			cntV++;
		}

	}


	copter_voltage = VOLTAGE_SCALE * (((sumV / cntV) * 3300.0) / 4096.0 );
	copter_current = CURRENT_SCALE * (((sumV / cntI) * 3300.0) / 4096.0 );

	copter_power = copter_voltage * copter_current;
}
// принимаем данные в прерывании на пине IRQ
uint8_t readData = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == NRF_IRQ_Pin){
		//printf("irq\n");

		NRF24_read(&readData, sizeof(readData));
		printf("irq: readData: 0x%x\n\r", readData);
		// если id бортового модуля не установлен -- записываем в него поле id
		// из принятого сообщения, т.е. выбираем данный модуль
		if (copter_id == 0){
			copter_id = message.id;
		}
		// если сообщение пришло от выбранного модуля -- записываем ток и напряжение
		// в массив для фильтрации, а также увеливаем счетчик принятых пакетов
		// чтобы вовремя отфильтровать значения
		if (message.id == copter_id){
			// фильтрация
			if (currentIndex <= FILTER_SIZE){
				receivedCurrents[currentIndex] = message.I_out_int;
				receivedVoltages[currentIndex] = message.V_out_int;
				currentIndex += 1;
			}else {
				currentIndex = 0;
				Filter();
			}
		}
		// если пришло сообщение от не выбранного модуля -- пропускаем его

	}

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*-------------- ПИД контроллер --------------*/
void PID_Control(PID_Param_e param, PID_t *coeffs)
{
	if (param == Input_Power) {
		coeffs->Kp = 5.0f;
		coeffs->Ki = 0.0f;
		coeffs->Kd = 20.0f;
		input = copter_power;
		ref = 165.0;
	}
	else if (param == Output_Current) {
		coeffs->Kp = 180.0f;
		coeffs->Ki = 0.0f;
		coeffs->Kd = 500.0f;
		input = copter_current;
		ref = TARGET_CURRENT;
	}
	// Вычисление ошибки
	error = ref - input;

	// Удержание ошибки в рамках значений, имеющих физический смысл

	// Пропорциональное слагаемое
	P = coeffs->Kp * error;

	// Интегральное слагаемое
	integral += error;

	I = coeffs->Ki * integral;

	// Дифференциальное слагаемое
	D = coeffs->Kd * (error - previousError);

	/* Выходное значение ПИД контроллера. Интегральное слагаемое не используется для избежания накопления ошибки */
	output = P + I + D;

	// Обновление значени ошибки
	previousError = error;

	// Корректировка фазового сдвига
	shift += (int16_t)output;

	// Удержание значение фазового сдвига в рамках значений, имеющих физический смысл
	if (shift > ((period_CHRG / 2) - 0x0080)) shift = (period_CHRG / 2 - 2 * 0x0080);
	else if (shift < 0x0080) shift = 0;
	shift_normal = shift;

	// Установка скорректированного значений фазового сдвига
	if (shift_normal > 0) {
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, (period_CHRG / 2 - 0x0080 - shift_normal));
	}
}
/*-------------- Функция инициализации приемопередатчика --------------*/
void NRF24_Init(void)
{
	NRF24_begin(hspi1);
	//nrf24_DebugUART_Init(huart2);
	NRF24_setDataRate(RF24_250KBPS);
	NRF24_setCRCLength(RF24_CRC_8);
	//NRF24_setPALevel(RF24_PA_0dB);
	NRF24_setChannel(0x6f);
	NRF24_setPayloadSize(1);
	NRF24_setAutoAck(0);
	NRF24_powerUp();
	NRF24_openReadingPipe(0, pipe);
	//NRF24_startListening();

   // printRadioSettings();
}

/*-------------- Функция установления частоты управляющего инвертором сигнала --------------*/
void HRTIM_Init(uint32_t period)
{
	__HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, period);
	__HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, period);
	__HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, period);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, period / 2 - 0x0080);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, period / 2);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, period / 2);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, period / 2);

	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
}

/*-------------- Функция изменения фазового сдвига --------------*/
void Change_Shift(void)
{
	__HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER, period_CHRG);
	__HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, period_CHRG);
	__HAL_HRTIM_SETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, period_CHRG);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, period_CHRG / 2);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, period_CHRG / 2);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, period_CHRG / 2);

	// фазовый сдвиг устанавливается согласно прцедуре ПИД контроллера
	PID_Control(PID_TYPE, &coeffs);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, (period_CHRG / 2 - 0x0080 - shift_normal));

}
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */
  GPIO_Set(LED_MCU_ON_GPIO_Port, LED_MCU_ON_Pin);
  printf("HAL is inited. Init NRF...");

	/*-------------- Инициализация приемопередатчика --------------*/
	DelayMicro(5000);
	NRF24_Init();
	NRF24_flush_tx();
	printConfigReg();
	printStatusReg();
	printFIFOstatus();
	bool isP = NRF24_isNRF_Plus();
	DelayMicro(1000);
	printf("Done!\n");

	// Установление частоты управляющего инвертором сигнала
	HRTIM_Init(period_WAIT);
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //printf("Main loop start");
	// Основной цикл программы
	while (1)
	{	// секция БЕЗ прерываний для считывания показаний датчиком станции
		__disable_irq();
		// Считывание тока и напряжения зарядной станции
		station_current = ACS712_ReadCurrentDC(&hadc1);
		station_voltage = myVoltageRead(&hadc1);

		// Проверка на заданные диапазоны тока и напряжения зарядной станции
		if ((station_current > ACS712_20A_MAX_CURRENT)||(station_current < ACS712_20A_MIN_CURRENT)){
			GPIO_Set(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
		}
		else{
			GPIO_Reset(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
		}


		if ((station_voltage > MAX_VOLTAGE)||(station_voltage < MIN_VOLTAGE)){
			GPIO_Set(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
		}
		else{
			GPIO_Reset(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
		}

		// Вывод информации по датчику
		//ACS712_DebugPrintf(&hadc1);
		// при нажатии на кнопку останавливаем таймер
		// при отжатии -- включаем
		int buttonStopState = pinReadDebounce(BUTTON_STOP_GPIO_Port, BUTTON_STOP_Pin, IS_BUTTON_PULLED_UP);
		//printf("\n\tDebug button stop: state %i", buttonStopState);
		int gerconState = pinReadDebounce(GERCON_IN_GPIO_Port, GERCON_IN_Pin, IS_BUTTON_PULLED_UP);
		//printf("\n\tDebug gercon: state %i", gerconState);

		__enable_irq();

		// ждем секунду для считывания пакета данных с бортового модуля
		NRF24_startListening();
		HAL_Delay(1000);
//		NRF24_stopListening();

		int dataIsHere = NRF24_available();
//
//		if (dataIsHere){
//			uint8_t readData = 0;
//			NRF24_read(&readData, sizeof(readData));
//			printf("data: %d", readData);
//			HAL_Delay(50);
//		}
//		printf("Message id: %li\n", message.id);
//		printf("Message i: %li\n", message.I_out_int);
//		printf("Message v: %li\n", message.V_out_int);

		if (buttonStopState){
		//	printf("\n\t Button stop pressed, disable hrtim");
			__HAL_HRTIM_DISABLE(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
			__HAL_HRTIM_DISABLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
			__HAL_HRTIM_DISABLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E);
		}else{
//			printf("\n\t Button stop not pressed, enable hrtim");
			__HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
			__HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
			__HAL_HRTIM_ENABLE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E);
		}



		if (gerconState){
			// Если геркон нажат -- дрон на месте, заряжаем
			printf("\n\tGercon is pressed");

			// Проверка на заданные диапазоны тока и напряжения коптера
			if ((copter_current > MAX_CURRENT) | (copter_current < MIN_CURRENT)) {
				//state = WAIT;
				printf("\n\tCopter current are out of range, waiting 1 minute...");
				GPIO_Set(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
				HRTIM_Init(period_WAIT);
				__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, (period_WAIT / 2 - 0x0080));
				GPIO_Reset(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
				DelayMicro(60 * 1000000);
				printf("\n\tDone");
			}

			if ((copter_voltage > MAX_CHARGE) | (copter_voltage < MIN_CHARGE)) {
				//state = WAIT;
				printf("\n\tCopter voltage are out of range, waiting 1 minute...");
				GPIO_Set(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
				GPIO_Reset(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
				DelayMicro(60 * 1000000);
				printf("\n\tDone");
				}

			if ((copter_current > MIN_CURRENT) & (copter_current < MAX_CURRENT) & (copter_voltage > MIN_CHARGE) & (copter_voltage < MAX_CHARGE))
			{
				//state = CHRG;
				printf("\n\t State: Charge");
				HRTIM_Init(period_CHRG);
				__disable_irq();
				Change_Shift();
				__enable_irq();

				HAL_GPIO_TogglePin(LED_MCU_STATE_GPIO_Port, LED_MCU_STATE_Pin);
				HAL_Delay(500);
			}
			else
			{
				// процедура ожидания
				//state = WAIT;

				printf("\n\t State: WAIT");
				HRTIM_Init(period_WAIT);
				GPIO_Set(LED_MCU_STATE_GPIO_Port, LED_MCU_STATE_Pin);
			}

		}
		else{
			// если коптер улетел и геркон не нажат -- обнуляем переменную-id
			copter_id = 0;
		}
		// задержка между итерациями
		HAL_Delay(500);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_SYSFLT;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0x675f;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0x0080;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0xFFEF;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0x675F;
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_DMABURST;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP1;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP2;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 0x3e80;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
  pDeadTimeCfg.RisingValue = 0x04f;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 0x04f;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.RisingValue = 0x02f;
  pDeadTimeCfg.FallingValue = 0x02f;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMPER;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0x7CFF;
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_SS_GPIO_Port, NRF_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_MCU_ON_Pin|LED_MCU_ERROR_Pin|LED_MCU_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRF_SS_Pin */
  GPIO_InitStruct.Pin = NRF_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GERCON_IN_Pin */
  GPIO_InitStruct.Pin = GERCON_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GERCON_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_STOP_Pin */
  GPIO_InitStruct.Pin = BUTTON_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_STOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MCU_ON_Pin LED_MCU_ERROR_Pin LED_MCU_STATE_Pin */
  GPIO_InitStruct.Pin = LED_MCU_ON_Pin|LED_MCU_ERROR_Pin|LED_MCU_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
	GPIO_Set(LED_MCU_ERROR_GPIO_Port, LED_MCU_ERROR_Pin);
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

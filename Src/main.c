/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Errors.h"
#include "SCPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_RECEIVE_BUFFER_SIZE 			(128u)
#define MAX_SEND_BUFFER_SIZE 				(512u)
#define MAX_SCPI_ANSWER_BUFFER_SIZE 		(512u)

#define PI 3.141592654

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId FreeRTOS_1msHandle;
osThreadId FreeRTOS_10msHandle;
osThreadId FreeRTOS_100msHandle;
osThreadId FreeRTOS_1000msHandle;
/* USER CODE BEGIN PV */
uint32_t Microseconds = 0;
uint8_t PWM_Duty;
uint32_t PWM_Freq;
uint32_t PWM_Prescaler;

uint16_t control_reg[1];
uint16_t freq0_reg_lsb[1] = {0x4000};
uint16_t freq0_reg_msb[1] = {0x4000};
uint16_t freq1_reg_lsb[1] = {0x8000};
uint16_t freq1_reg_msb[1] = {0x8000};
uint16_t phase0_reg[1];
uint16_t exit_reg[1];

uint32_t freq;
uint32_t freq_reg_value;

signal_type_T signal_type;
signal_type_T setSignalType;
uint32_t setSignalFreq;
uint8_t setSignalFlag = 0;

uint16_t dacStep;
dac_signal_type_T dacSignalSelect;


uint16_t gauss_value;
uint16_t gauss_mean;
uint16_t gauss_std_dev;

multiplexer_select_T multiplexerChannelSelect;

uint8_t receive_buffer[MAX_RECEIVE_BUFFER_SIZE] = "";
uint8_t send_buffer[MAX_SEND_BUFFER_SIZE] = "";
uint8_t SCPI_buffer[MAX_SCPI_ANSWER_BUFFER_SIZE] = "";
volatile uint8_t receiveByte = '\0';

uint32_t i = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void RTOS_1ms(void const * argument);
void RTOS_10ms(void const * argument);
void RTOS_100ms(void const * argument);
void RTOS_1000ms(void const * argument);

/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_Delay10Microseconds(uint32_t DelayMicroseconds);
void GenerateAD9833Signal(signal_type_T, uint32_t);
void SetSignalType(signal_type_T);
void CalculateFrequency(uint32_t);
void GeneratePWMSignal(uint8_t, uint32_t);
void MultiplexerChannelSelect(multiplexer_select_T);

double gaussrand(void);
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
	control_reg[0] = 0x2100;
	phase0_reg[0] = 0xC000;

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
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, &receiveByte, 1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  PWM_Freq = 10000;
  PWM_Duty = 25;
  setSignalFreq = 5000;

  gauss_mean = 2048;
  gauss_std_dev = 500;

  GeneratePWMSignal(PWM_Duty, PWM_Freq);
  GenerateAD9833Signal(SIN, setSignalFreq);
  MultiplexerChannelSelect(AD9833);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of FreeRTOS_1ms */
  osThreadDef(FreeRTOS_1ms, RTOS_1ms, osPriorityNormal, 0, 128);
  FreeRTOS_1msHandle = osThreadCreate(osThread(FreeRTOS_1ms), NULL);

  /* definition and creation of FreeRTOS_10ms */
  osThreadDef(FreeRTOS_10ms, RTOS_10ms, osPriorityNormal, 0, 128);
  FreeRTOS_10msHandle = osThreadCreate(osThread(FreeRTOS_10ms), NULL);

  /* definition and creation of FreeRTOS_100ms */
  osThreadDef(FreeRTOS_100ms, RTOS_100ms, osPriorityNormal, 0, 128);
  FreeRTOS_100msHandle = osThreadCreate(osThread(FreeRTOS_100ms), NULL);

  /* definition and creation of FreeRTOS_1000ms */
  osThreadDef(FreeRTOS_1000ms, RTOS_1000ms, osPriorityNormal, 0, 128);
  FreeRTOS_1000msHandle = osThreadCreate(osThread(FreeRTOS_1000ms), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MULTIPLEXER_S0_Pin|MULTIPLEXER_S1_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : MULTIPLEXER_S0_Pin MULTIPLEXER_S1_Pin PB8 */
  GPIO_InitStruct.Pin = MULTIPLEXER_S0_Pin|MULTIPLEXER_S1_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		ERROR_CODE errorCode = 0x00;
		static uint16_t counter = 0;
		static uint8_t FLAG_needsCleaning = 0;
		uint16_t size = 0;
		char sin[] = "SIN";
		char tri[] = "TRI";

		if (1 == FLAG_needsCleaning)
		{
			memset(receive_buffer,'\0',(MAX_RECEIVE_BUFFER_SIZE*sizeof(receive_buffer[0])));
			memset(send_buffer,'\0',(MAX_SEND_BUFFER_SIZE*sizeof(send_buffer[0])));
			memset(SCPI_buffer,'\0',(MAX_SCPI_ANSWER_BUFFER_SIZE*sizeof(SCPI_buffer[0])));
			FLAG_needsCleaning = 0;
		}

		if ('\r' != receiveByte && '\n' != receiveByte)
		{
			if (counter <= MAX_RECEIVE_BUFFER_SIZE)
			{
				receive_buffer[counter] = receiveByte;
			}
			++counter;
		}

		else
		{
			errorCode = Process_SCPI_Message(receive_buffer, SCPI_buffer);
			if(0 == errorCode)
				size = sprintf(send_buffer, "Message correct!: \"%s\".\n\r%s\n\r", receive_buffer,SCPI_buffer);
			else
				size = sprintf(send_buffer, "!!Message incorrect!! \n\rError Code: (%d) %s \n\rReceived message: \"%s\"\n\r", errorCode, Return_Error_Description(errorCode), receive_buffer);

			HAL_UART_Transmit_IT(&huart3, send_buffer, size);
			counter = 0;
			FLAG_needsCleaning = 1;
		}
		HAL_UART_Receive_IT(&huart3, &receiveByte, 1);
}

void GenerateAD9833Signal(signal_type_T signal_type, uint32_t freq)
{
	SetSignalType(signal_type);
	CalculateFrequency(freq);

    exit_reg[0] = control_reg[0] & 0xEFF;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)control_reg, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)freq0_reg_lsb, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)freq0_reg_msb, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)phase0_reg, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)exit_reg, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(10);
}

void SetSignalType(signal_type_T signal_type)
{
  if (signal_type == SIN)
  {
    control_reg[0] &= ~(0b100010);
  }
  else if (signal_type == TRIANGLE)
  {
    control_reg[0] &= ~(0b100000);
    control_reg[0] |= 0b10;
  }
  else if (signal_type == SQUARE)
  {
    control_reg[0] &= ~(0b10);
    control_reg[0] |= 0b101000;
  }
}

void CalculateFrequency(uint32_t freq)
{
	freq_reg_value = (uint32_t)freq * 10.737;
	freq0_reg_lsb[0] = (freq_reg_value & ~(0xFFFC000));
	freq0_reg_lsb[0] |= 0x4000;
	freq0_reg_lsb[0] &= ~(0x8000);
	freq0_reg_msb[0] = (freq_reg_value & 0xFFFC000) >> 14;
	freq0_reg_msb[0] |= 0x4000;
	freq0_reg_msb[0] &= ~(0x8000);
}

void GeneratePWMSignal(uint8_t duty, uint32_t freq)
{
	PWM_Prescaler = (640000/freq) - 1;
	TIM1->PSC = PWM_Prescaler;
	TIM1->CCR3 = duty;
}

void MultiplexerChannelSelect(multiplexer_select_T channel)
{
	switch(channel)
	{
		case AD9833:
		{
			HAL_GPIO_WritePin(MULTIPLEXER_S0_GPIO_Port, MULTIPLEXER_S0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MULTIPLEXER_S1_GPIO_Port, MULTIPLEXER_S1_Pin, GPIO_PIN_RESET);
			break;
		}
		case CA_CH1:
		{
			HAL_GPIO_WritePin(MULTIPLEXER_S0_GPIO_Port, MULTIPLEXER_S0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MULTIPLEXER_S1_GPIO_Port, MULTIPLEXER_S1_Pin, GPIO_PIN_RESET);
			break;
		}
		case PWM:
		{
			HAL_GPIO_WritePin(MULTIPLEXER_S0_GPIO_Port, MULTIPLEXER_S0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MULTIPLEXER_S1_GPIO_Port, MULTIPLEXER_S1_Pin, GPIO_PIN_SET);
			break;
		}
	}
}

double gaussrand()
{
    static double U, V;
    static int phase = 0;
    double Z;

    if(phase == 0) {
        U = (rand() + 1.) / (RAND_MAX + 2.);
        V = rand() / (RAND_MAX + 1.);
        Z = sqrt(-2 * log(U)) * sin(2 * PI * V);
    } else
        Z = sqrt(-2 * log(U)) * cos(2 * PI * V);

    phase = 1 - phase;

    return Z;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(SAWTOOTH == dacSignalSelect)
		{
			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i);
			  i = i + dacStep;
			  if (i >= 4095)
			  {
				  i = 0;
			  }
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    
    
    
    
    
    

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if (setSignalFlag == 1) {
	  		  if (multiplexerChannelSelect == AD9833)
	  		  {
	  			  GenerateAD9833Signal(setSignalType, setSignalFreq);
	  			  MultiplexerChannelSelect(multiplexerChannelSelect);
	  		  }
	  		  else if (multiplexerChannelSelect == CA_CH1)
	  		  {
	  			  MultiplexerChannelSelect(multiplexerChannelSelect);
	  		  }
	  		  else if (multiplexerChannelSelect == PWM)
	  		  {
	  			  GeneratePWMSignal(PWM_Duty, PWM_Freq);
	  			  MultiplexerChannelSelect(multiplexerChannelSelect);
	  		  }
	  		  setSignalFlag = 0;
	  		}

	  	  if (GAUSSIAN == dacSignalSelect)
	  	  {
	  		  gauss_value = gauss_std_dev * gaussrand() + gauss_mean;
	  		  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, gauss_value);
	  	  }
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_RTOS_1ms */
/**
* @brief Function implementing the FreeRTOS_1ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_1ms */
void RTOS_1ms(void const * argument)
{
  /* USER CODE BEGIN RTOS_1ms */

	const TickType_t xFrequency = 1;

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay( xFrequency / portTICK_RATE_MS );
  }
  /* USER CODE END RTOS_1ms */
}

/* USER CODE BEGIN Header_RTOS_10ms */
/**
* @brief Function implementing the FreeRTOS_10ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_10ms */
void RTOS_10ms(void const * argument)
{
  /* USER CODE BEGIN RTOS_10ms */

	const TickType_t xFrequency = 10;

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay( xFrequency / portTICK_RATE_MS );
  }
  /* USER CODE END RTOS_10ms */
}

/* USER CODE BEGIN Header_RTOS_100ms */
/**
* @brief Function implementing the FreeRTOS_100ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_100ms */
void RTOS_100ms(void const * argument)
{
  /* USER CODE BEGIN RTOS_100ms */

	const TickType_t xFrequency = 100;

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay( xFrequency / portTICK_RATE_MS );
  }
  /* USER CODE END RTOS_100ms */
}

/* USER CODE BEGIN Header_RTOS_1000ms */
/**
* @brief Function implementing the FreeRTOS_1000ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTOS_1000ms */
void RTOS_1000ms(void const * argument)
{
  /* USER CODE BEGIN RTOS_1000ms */

	const TickType_t xFrequency = 1000;

  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay( xFrequency / portTICK_RATE_MS );
  }
  /* USER CODE END RTOS_1000ms */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

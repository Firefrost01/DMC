/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @author					: polyakovyg
	* @revision				: 01.08.02
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include "delayus.h"
#include "task.h"
#include "ds18b20.h"
#include "math.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskNRF24 */
osThreadId_t TaskNRF24Handle;
const osThreadAttr_t TaskNRF24_attributes = {
  .name = "TaskNRF24",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TaskCore */
osThreadId_t TaskCoreHandle;
const osThreadAttr_t TaskCore_attributes = {
  .name = "TaskCore",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TaskINA226 */
osThreadId_t TaskINA226Handle;
const osThreadAttr_t TaskINA226_attributes = {
  .name = "TaskINA226",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskLog */
osThreadId_t TaskLogHandle;
const osThreadAttr_t TaskLog_attributes = {
  .name = "TaskLog",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for QueueGPIORC */
osMessageQueueId_t QueueGPIORCHandle;
const osMessageQueueAttr_t QueueGPIORC_attributes = {
  .name = "QueueGPIORC"
};
/* Definitions for QueueLog */
osMessageQueueId_t QueueLogHandle;
const osMessageQueueAttr_t QueueLog_attributes = {
  .name = "QueueLog"
};
/* Definitions for QueueInaTx */
osMessageQueueId_t QueueInaTxHandle;
const osMessageQueueAttr_t QueueInaTx_attributes = {
  .name = "QueueInaTx"
};
/* Definitions for Spi1TxBinarySem */
osSemaphoreId_t Spi1TxBinarySemHandle;
const osSemaphoreAttr_t Spi1TxBinarySem_attributes = {
  .name = "Spi1TxBinarySem"
};
/* Definitions for Spi2TxBinarySem */
osSemaphoreId_t Spi2TxBinarySemHandle;
const osSemaphoreAttr_t Spi2TxBinarySem_attributes = {
  .name = "Spi2TxBinarySem"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void StartTaskNRF24(void *argument);
void StartTaskCore(void *argument);
void StartTaskINA226(void *argument);
void StartTaskLog(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void vSetUpDebug(void);

#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000


struct __FILE { int handle; 
	// Add whatever you need here  
	};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) {
		while (ITM_Port32(0) == 0){};
    ITM_Port8(0) = ch;
  }
  return(ch);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	
	if( HAL_SPI_GetError( hspi ) ) {
		assert_param( 1u );
	}

	else if( hspi == &hspi2 ) {
		#if( _SPI_SEMAPHORE_ == 1 )
			xSemaphoreGiveFromISR( Spi2TxBinarySemHandle , &xHigherPriorityTaskWoken );
		#elif( _SPI_NOTIFICATION_ == 1 )
			xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(TaskNRF24Handle , &xHigherPriorityTaskWoken);
		#endif // _SPI_SEMAPHORE_ _SPI_NOTIFICATION_
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	
	if( HAL_SPI_GetError( hspi ) ) {
		assert_param( 1u );
	}

	else if( hspi == &hspi2 ) {
		#if( _SPI_SEMAPHORE_ == 1 )
			xSemaphoreGiveFromISR( Spi2TxBinarySemHandle , &xHigherPriorityTaskWoken );
		#elif( _SPI_NOTIFICATION_ == 1 )
			xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(TaskNRF24Handle , &xHigherPriorityTaskWoken);
		#endif // _SPI_SEMAPHORE_ _SPI_NOTIFICATION_
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if( HAL_SPI_GetError( hspi ) ) {
		assert_param( 1u );
	}
}

//DS18B20_INIT();


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
	void vInitDebug();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Spi1TxBinarySem */
  Spi1TxBinarySemHandle = osSemaphoreNew(1, 0, &Spi1TxBinarySem_attributes);

  /* creation of Spi2TxBinarySem */
  Spi2TxBinarySemHandle = osSemaphoreNew(1, 0, &Spi2TxBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueGPIORC */
  QueueGPIORCHandle = osMessageQueueNew (1, sizeof(uint16_t), &QueueGPIORC_attributes);

  /* creation of QueueLog */
  QueueLogHandle = osMessageQueueNew (16, sizeof(uint16_t), &QueueLog_attributes);

  /* creation of QueueInaTx */
  QueueInaTxHandle = osMessageQueueNew (1, sizeof(ina_data_t), &QueueInaTx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TaskNRF24 */
  TaskNRF24Handle = osThreadNew(StartTaskNRF24, NULL, &TaskNRF24_attributes);

  /* creation of TaskCore */
  TaskCoreHandle = osThreadNew(StartTaskCore, NULL, &TaskCore_attributes);

  /* creation of TaskINA226 */
  TaskINA226Handle = osThreadNew(StartTaskINA226, NULL, &TaskINA226_attributes);

  /* creation of TaskLog */
  TaskLogHandle = osThreadNew(StartTaskLog, NULL, &TaskLog_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|RELAY_1_Pin|RELAY_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NRF_CSN_Pin|NRF_CE_Pin|ONE_WIRE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin RELAY_1_Pin RELAY_2_Pin */
  GPIO_InitStruct.Pin = LED_Pin|RELAY_1_Pin|RELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT_L_Pin */
  GPIO_InitStruct.Pin = BUT_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUT_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT_OK_Pin BUT_R_Pin */
  GPIO_InitStruct.Pin = BUT_OK_Pin|BUT_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INA_INT_Pin */
  GPIO_InitStruct.Pin = INA_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INA_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CSN_Pin NRF_CE_Pin ONE_WIRE_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin|NRF_CE_Pin|ONE_WIRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		for (uint32_t x = 0; x < 0x0000FFFF; x++) __NOP();
    //osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskNRF24 */
/**
* @brief Function implementing the TaskNRF24 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskNRF24 */
void StartTaskNRF24(void *argument)
{
  /* USER CODE BEGIN StartTaskNRF24 */
	uint8_t counter = 0;
	
	uint8_t RxAddress[] = {0x04,0xD5,0xC6,0xB7,0xA8};
	uint8_t TxAddress[] = {0xE8,0xD7,0xC6,0xB5,0xA4};
	
	nrf_data_t NrfTxData = {0,};
	uint8_t NrfRxData[8] = {0,};
	
	#ifdef DS18B20_TASK
	extern osMessageQueueId_t QueueDS18B20Handle;	
	Ds18b20Sensor_t TemperatureData;
	#endif //DS18B20_TASK
	
	#ifdef INA226_TASK
	ina_data_t InaData = {0,};	
	#endif //INA226_TASK

  /* Infinite loop */
  for(;;)
  {
		NRF24_RxMode(RxAddress, 10);

		//---GPIO_RC--------------------------------------------------------// 
		#ifdef GPIO_RC
			if((uint8_t)uxQueueMessagesWaiting(QueueGPIORCHandle))
			{
				if (isDataAvailable(2) == 1)
				{
					for(counter = 0; counter < 2; counter++)
					{
						NRF24_Receive(&NrfRxData[counter]);
					}
					
					if(NrfTxData.Data[0] == NRF_GPIO_RELAY)
					{
						HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, (GPIO_PinState)(NrfRxData[1]));
						HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, (GPIO_PinState)(NrfRxData[2]));
					}
				}
			}
		#endif //GPIO_RC
		//---end GPIO_RC--------------------------------------------------------// 
			
		NRF24_TxMode(TxAddress, 10);
			
		//---DS18B20--------------------------------------------------------//
		#ifdef DS18B20_TASK
			if((uint8_t)uxQueueMessagesWaiting(QueueDS18B20Handle))
			{
				xQueueReceive(QueueDS18B20Handle, &TemperatureData, pdMS_TO_TICKS(100));
				if(TemperatureData.DataIsValid)
					{
					NrfTxData.Data[0] = NRF_TEMP_SENSOR;
					NrfTxData.Data[1] = TemperatureData.Temperature;
						
					//printf( "DS18B20 ID: %x  ", TemperatureData.Address );
					//printf( "Temperature: %5.2f C\n", TemperatureData.Temperature );
					
					if (xQueueSend(QueueLogHandle, &NrfTxData.Data[1], portMAX_DELAY) != pdPASS)
					{
						assert_param( 1u );
					}
					
					for(counter = 0; counter < 2; counter++)
					{
						NRF24_Transmit(&NrfTxData.Data[counter]);
					}
				} else 	{
					printf( "DS18B20 sensor lost" );
				}
			}
		#endif //DS18B20_TASK
		//---end DS18B20--------------------------------------------------------//

		//---BUTTONS--------------------------------------------------------//
		#ifdef BUTTONS_TASK
			if((uint8_t)uxQueueMessagesWaiting(QueueButtonHandle))
			{
				xQueueReceive(QueueButtonHandle, &buttonState, pdMS_TO_TICKS(100));
				
				NrfTxData.Data[0] = NRF_BUTTONS;
				for(counter = 1; counter < 4; counter++)	
				{
					NrfTxData.Data[i] = buttonState[i];
				}
				
				if (xQueueSend(QueueLogHandle, NrfTxData, portMAX_DELAY) != pdPASS)
				{
					assert_param( 1u );
				}

				for(counter = 0; counter < 4; counter++)
				{
					NRF24_Transmit(NrfTxData.Data[counter]);
				}
			}
		#endif //BUTTONS_TASK
		//---end BUTTONS--------------------------------------------------------//

		//---end INA226--------------------------------------------------------//
		#ifdef INA226_TASK
			if((uint8_t)uxQueueMessagesWaiting(QueueInaTxHandle))
			{
				xQueueReceive(QueueInaTxHandle, &InaData, pdMS_TO_TICKS(100));

				NrfTxData.Data[0] = NRF_CURRENT_SENSOR;
				NrfTxData.Data[1] = InaData.BusVoltage;
				NrfTxData.Data[2] = InaData.Current;
				NrfTxData.Data[3] = InaData.Voltage;
				NrfTxData.Data[4] = InaData.Power;
				
				if (xQueueSend(QueueLogHandle, &NrfTxData.Data[4], portMAX_DELAY) != pdPASS)
				{
					assert_param( 1u );
				}

				for(counter = 0; counter < 5; counter++)
				{
					NRF24_Transmit(&NrfTxData.Data[counter]);
				}
			}
		#endif //INA226_TASK
		//---end INA226--------------------------------------------------------//

    osDelay(1000);
  }
  /* USER CODE END StartTaskNRF24 */
}

/* USER CODE BEGIN Header_StartTaskCore */
/**
* @brief Function implementing the TaskCore thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCore */
void StartTaskCore(void *argument)
{
  /* USER CODE BEGIN StartTaskCore */
	
	#ifdef BUTTON_TASK
		BUTTON_Init();
		uint16_t GPIO_Pin = 0;
		uint8_t button_state = 0;
	
		HAL_TIM_Base_Start_IT(@htim2);
	#endif //BUTTON_TASK
	
  /* Infinite loop */
  for(;;)
  {
		//---BUTTON--------------------------------------------------------//
		#ifdef BUTTON_TASK
			if((uint8_t)uxQueueMessagesWaiting(QueueButtonHandle))
			{
				xQueueReceive(QueueButtonHandle, &button_action, pdMS_TO_TICKS(100));
				if (button_action.left == BUTTON_SHORT_PRESS)
				{
					//do
				}
				if (button_action.left == BUTTON_LONG_PRESS || left_button_action == BUTTON_VERY_LONG_PRESS)
				{
					//do
				}

				if (button_action.right == BUTTON_SHORT_PRESS)
				{
					//do
				}
				if (button_action.right == BUTTON_LONG_PRESS || right_button_action == BUTTON_VERY_LONG_PRESS)
				{
					//do
				}

				if (button_action.ok == BUTTON_SHORT_PRESS)
				{
					//do nothing
				}
				if (button_action.ok == BUTTON_LONG_PRESS || ok_button_action == BUTTON_VERY_LONG_PRESS)
				{
					//do nothing
				}
				
				BUTTON_ResetActions();
				
				if (xQueueSend(QueueNrfTransmitHandle, button_action, pdMS_TO_TICKS(100)) != pdPASS)
				{
					assert_param( 1u );
				}
			}					
		#endif //BUTTON_TASK
		//---end BUTTON--------------------------------------------------------//

    osDelay(100);
  }
  /* USER CODE END StartTaskCore */
}

/* USER CODE BEGIN Header_StartTaskINA226 */
/**
* @brief Function implementing the TaskINA226 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskINA226 */
void StartTaskINA226(void *argument)
{
  /* USER CODE BEGIN StartTaskINA226 */
	ina_data_t InaData = {0,};
	INA226_setConfig(&INA226_I2C, INA226_ADDRESS, INA226_MODE_CONT_SHUNT_AND_BUS | INA226_VBUS_140uS | INA226_VBUS_140uS | INA226_AVG_1024);
	
  /* Infinite loop */
  for(;;)
  {
		InaData.BusVoltage = INA226_getBusV(&hi2c2, INA226_ADDRESS);
		InaData.Current = INA226_getCurrent(&hi2c2, INA226_ADDRESS);
		InaData.Voltage = INA226_getShuntV(&hi2c2, INA226_ADDRESS);
		InaData.Power = INA226_getPower(&hi2c2, INA226_ADDRESS);
		
		if ( InaData.BusVoltage > 1000.0F) 
		{
			if (xQueueSend(QueueInaTxHandle, &InaData, pdMS_TO_TICKS(100)) != pdPASS)
				{
					assert_param( 1u );
				}
			//printf( "Voltage: %d V\n", InaData.BusVoltage );
		}
		else 
		{
			printf( "INA is Not connected. \n");
		}
    osDelay(1000);
  }
  /* USER CODE END StartTaskINA226 */
}

/* USER CODE BEGIN Header_StartTaskLog */
/**
* @brief Function implementing the TaskLog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLog */
void StartTaskLog(void *argument)
{
  /* USER CODE BEGIN StartTaskLog */
//	#ifdef FLASH_TASK
//	log_data_t LogData;
//	#endif //FLASH_TASK
  /* Infinite loop */
  for(;;)
  {
//		#ifdef FLASH_TASK
//			xQueueReceive(QueueLogHandle, &LogData, portMAX_DELAY);			
//			//HAL_SPI_Transmit(&FLASH_SPI, (uint8_t*)&LogData, 1, 100);
//			printf( "LogData: %d %s %d %d \n", LogData.LogType, LogData.Text, LogData.Date, LogData.Time);
//		#endif //FLASH_TASK
		
    osDelay(1000);
  }
  /* USER CODE END StartTaskLog */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

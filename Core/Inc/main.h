/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "debug.h"
#include "INA226.h"
#include "button.h"
#include "NRF24L01.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define RELAY_1_Pin GPIO_PIN_14
#define RELAY_1_GPIO_Port GPIOC
#define RELAY_2_Pin GPIO_PIN_15
#define RELAY_2_GPIO_Port GPIOC
#define AIN_1_Pin GPIO_PIN_0
#define AIN_1_GPIO_Port GPIOA
#define BUT_L_Pin GPIO_PIN_1
#define BUT_L_GPIO_Port GPIOA
#define BUT_L_EXTI_IRQn EXTI1_IRQn
#define BUT_OK_Pin GPIO_PIN_0
#define BUT_OK_GPIO_Port GPIOB
#define BUT_OK_EXTI_IRQn EXTI0_IRQn
#define INA_INT_Pin GPIO_PIN_2
#define INA_INT_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_12
#define NRF_CSN_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_4
#define NRF_CE_GPIO_Port GPIOB
#define BUT_R_Pin GPIO_PIN_8
#define BUT_R_GPIO_Port GPIOB
#define BUT_R_EXTI_IRQn EXTI9_5_IRQn
#define ONE_WIRE_Pin GPIO_PIN_9
#define ONE_WIRE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//---NRF--------------------------------------------------------//
#define _RF_NRF24L01_					1
#define _SPI_NOTIFICATION_		1
#define _SPI_SEMAPHORE_				0
#define NRF24_SPI 						hspi2
#define NRF_CS_Pin 						GPIO_PIN_12
#define NRF_CS_GPIO_Port 			GPIOB
#define NRF_CE_Pin 						GPIO_PIN_4
#define NRF_CE_GPIO_Port 			GPIOB
#define NRF_INTERRUPT_ON			0x1
typedef enum 
{
	NRF_NOSRS,
	NRF_TEMP_SENSOR,
	NRF_RFID_SENSOR,
	NRF_CURRENT_SENSOR,
	NRF_BUTTONS,
	NRF_GPIO_RELAY,
} nrf_txrx_e;
typedef struct
{
	uint8_t 		Data[8];
}nrf_data_t;
//#define NRF_TRANSMIT					0x1
//#define NRF_RECEIVE					  0x1
//---end NRF--------------------------------------------------------//

//---DS18B20--------------------------------------------------------//
#define DS18B20_TASK												1
#define	DS18B20_USE_FREERTOS		    				1
#define DS18B20_MAX_SENSORS		    					1
#define	DS18B20_CONVERT_TIMEOUT_MS					1000
#define	DS18B20_UPDATE_INTERVAL_MS					1000		
#define	DS18B20_GPIO												ONE_WIRE_GPIO_Port
#define	DS18B20_PIN													ONE_WIRE_Pin
#define	DS18B20_TIMER												htim1				
//---end DS18B20--------------------------------------------------------//

//---INA226--------------------------------------------------------//
#define INA226_TASK													1
#define INA226_I2C 													hi2c2
typedef struct
{
	uint16_t		Current;
	uint16_t 		BusVoltage;
	uint16_t		Voltage;
	uint16_t		Power;
}ina_data_t;
//---end INA226--------------------------------------------------------//

//---GPIO_RC--------------------------------------------------------//
#define	GPIO_RC		    											1
#define	GPIO_RC_QUANTITY										2
//---end GPIO_RC--------------------------------------------------------//

//---BUTTON--------------------------------------------------------//
#define BUTTONS_LONG_PRESS_MS								500
#define BUTTONS_VERY_LONG_PRESS_MS					3000
#define DEBOUNCE_TIME_MS										100
typedef enum 
{
	BUTTON_LEFT,
	BUTTON_RIGHT,
	BUTTON_OK,
	BUTTONS_NUM,
} ButtonID;
static McuPin buttons[BUTTONS_NUM] = {{BUT_L_GPIO_Port, BUT_L_Pin},
																			{BUT_R_GPIO_Port, BUT_R_Pin},
                                      {BUT_OK_GPIO_Port, BUT_OK_Pin}};
//---end BUTTON--------------------------------------------------------//

//---DEBUG--------------------------------------------------------//
//DEBUG
#define DEBUG_CHECK_IDLE_TIME					1
#define DEBUG_TASK_TERM_MS						500
#define DEBUG_TASK_PRIORITY						osPriorityLow
//---end DEBUG--------------------------------------------------------//
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

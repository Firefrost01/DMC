/**
  ******************************************************************************
  * @file           : button.h
  * @brief          : Buttons driver
  * @author         : TQFP (for https://microtechnics.ru/user-blogs/)
  ******************************************************************************
  */

#ifndef BUTTON_H
#define BUTTON_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"

/* Declarations and definitions ----------------------------------------------*/

#define GPIO_BUTTON_NOT_PRESSED                              (GPIO_PIN_SET)		//PULL-UP	- not connected
#define GPIO_BUTTON_PRESSED                                  (!GPIO_BUTTON_NOT_PRESSED)

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
	//GPIO_PinState pupd_pin;
} McuPin;

typedef enum
{
  BUTTON_STARTING                      = 0,
  BUTTON_NOT_PRESSED                   = 1,
  BUTTON_WAIT_DEBOUNCE                 = 2,
  BUTTON_PRESSED                       = 3,
} ButtonState;

typedef enum
{
  BUTTON_NONE                          = 0,
  BUTTON_SHORT_PRESS                   = 1,
  BUTTON_LONG_PRESS                    = 2,
  BUTTON_VERY_LONG_PRESS               = 3,
} ButtonAction;

/* Functions -----------------------------------------------------------------*/

extern void BUTTON_Process(void);
extern void BUTTON_TimerProcess(void);
extern ButtonAction BUTTON_GetAction(uint8_t index);
extern void BUTTON_ResetActions(void);
extern void BUTTON_Init(void);

#endif // #ifndef BUTTON_H

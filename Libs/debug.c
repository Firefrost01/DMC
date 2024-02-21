/******************** (C) COPYRIGHT  ********************
* File Name          : debug.c
* Author             : 
* Date First Issued  : 18/12/2023
* Description        : This file provides ...
********************************************************************************
* History:
* 17/12/2023: V0.1
********************************************************************************
* .
*******************************************************************************/

#include "debug.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

//#define TASK_DEBUG			1U
//#define IDLE_MON				1U

//void TaskSwitchedIn(uint8_t tag);
//{
//#ifdef TASK_DEBUG
//	switch( tag ) {
//		case 1:
//			GPIOA->BSSR = GPIO_PIN_0;
//			break;
//		case 2:
//			GPIOA->BSSR = GPIO_PIN_1;
//			break;
//		case 3:
//			GPIOA->BSSR = GPIO_PIN_2;
//			break;
//		case 4:
//			GPIOA->BSSR = GPIO_PIN_3;
//			break;
//	}
//#endif
//}
//
//void TaskSwitchedOut(uint8_t tag);
//{
//#ifdef TASK_DEBUG
//	switch( tag ) {
//		case 1:
//			GPIOA->BSSR = (uint32_t)GPIO_PIN_0 << 16U;
//			break;
//		case 2:
//			GPIOA->BSSR = (uint32_t)GPIO_PIN_1 << 16U;
//			break;
//		case 3:
//			GPIOA->BSSR = (uint32_t)GPIO_PIN_2 << 16U;
//			break;
//		case 4:
//			GPIOA->BSSR = (uint32_t)GPIO_PIN_3 << 16U;
//			break;
//	}
//#endif
//}
//
//void vApplicationIdleHook( void ) {
//	#ifdef IDLE_MON
//		GPIOA->BSSR = GPIO_PIN_4;
//		_NOP();
//		GPIOA->BSSR = (uint32_t)GPIO_PIN_4 << 16U;
//	#endif //IDLE_MON
//}

//place in head of task which be controled
//#ifdef TASK_DEBUG
//	vTaskSetApplicationTaskTag( NULL, (void*) 1 );
//#endif //TASK_DEBUG

//for(uint32_t x = 0; x < 0x2FFe; x++ ) __NOP();

//##################################################
//Init debug task.
/*
void vInitDebug(void *Parameters)
{
	#if (DEBUG_CHECK_IDLE_TIME==1)
		volatile uint32_t ulIdleCycleCount = 0UL;
		#ifndef DEBUG_TASK
			#define DEBUG_TASK 				1
		#endif
	#endif
	#if (DEBUG_TASK==1)
		void vStartDebugTask(void *argument);
		osThreadId_t 	DebugTaskHandle;
		const osThreadAttr_t DebugTask_attributes = {
			.name = "DebugTask",
			.stack_size = 128 * 1,
			.priority = (osPriority_t) DEBUG_TASK_PRIORITY,
		};
		DebugTaskHandle = osThreadNew(vStartDebugTask, NULL, &DebugTask_attributes);
	#endif
//!	return 0;
}

//Debug task.
#if (DEBUG_TASK==1)
void vStartDebugTask(void *argument)
{
	const TickType_t xDelayms = pdMS_TO_TICKS(DEBUG_TASK_TERM_MS);
	extern uint32_t ulIdleCycleCount;
	for(;;)
	{
		printf("Debug IDLE Time: %dms\n", ulIdleCycleCount);
		vTaskDelay(xDelayms);
	}
}
#endif

//IDLE Hook.
void functionIdleHook()
{
	#if (DEBUG_CHECK_IDLE_TIME==1)
		ulIdleCycleCount++;
	#endif
	//sleep for next interrupt.
	#if (configUSE_TICKLESS_IDLE==1)
		HAL_PWR_EnterSLEEPMode(NULL, PWR_SLEEPENTRY_WFI);
	#endif
}
*/
//Set up debug.
#if (DEBUG_PRINTF==1)
void vSetUpDebug()
{
	//printf.
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
	//end printf.
}
#endif

//portSUPRESS_TICKS_AND_SLEEP()
//configUSE_TICKLESS_IDLE		2	//0 - ,1 - internal function, 3 - user function
//All program timers should be stoped.
//All tasks blocked for portMAX_DELAY.
//configUSE_TICKLESS_IDLE should be set to 2
//prvGetExpectedIdleTime()

//Config internal function of tickless IDLE.
#if (configUSE_TICKLESS_IDLE == 1)
	#ifndef configEXPECTED_IDLE_TIME_BEFORE_SLEEP
		#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP 2
	#endif

//Pre sleep processing.
void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	// place for user code
}

//Post sleep processing.
void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	// place for user code 
}
#endif /* configUSE_TICKLESS_IDLE */

//!//void vApplicationSleep

//User function of tickless IDLE.
#if (configUSE_TICKLESS_IDLE == 2)
	void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
#endif

//##################################################
//Correcting ticks in critical section.
//Example/
/*
void vExampleFunction(void)
{
	unsigned long ulTimeBefore, ulTimeAfter;
	//Save time.
	ulTimeBefore = ulGetExternalTime();
	//Stop the tick's timer.
	pvrStopTickInterruptTimer();
	//Action.
	
	arbitary_processing();
	//Read time.
	ulTimeAfter = ulGetExternalTime();
	if(xTaskCatchUpTicks(ulTimeAfter - ulTimeBefore) == pdTRUE)
	{
		//Time correction call switch context.
	}
	//Reset Tick Timer.
	pvrStartTickInterruptTimer();
}
*/
/*-------------------------------------------------------------*/

/******************* (C) COPYRIGHT  *****END OF FILE****/

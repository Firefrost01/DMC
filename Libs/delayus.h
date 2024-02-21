#ifndef __DELAYUS_H__
#define __DELAYUS_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
	 
#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
#define SCB_DEMCR   *(volatile unsigned long *)0xE000EDFC

void DWT_Init(void);
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif
#endif //__DELAYUS_H__



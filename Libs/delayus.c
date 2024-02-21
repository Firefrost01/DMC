//DWT Timer

/*
//Use Example 
// main.h
#include "delayus.h"

//main() {
DWT_Init();

//Delay us
delay_us(100);
*/

#include "delayus.h"
#include "main.h"

void DWT_Init(void)
{
  SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // allow counter
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // stert counter
}

void delay_us(uint32_t us) // DelayMicro
{
    uint32_t us_count_tic =  us * (SystemCoreClock / 1000000);
    DWT->CYCCNT = 0U; // reset counter
    while(DWT->CYCCNT < us_count_tic);
}

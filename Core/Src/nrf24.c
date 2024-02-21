// nrf24l01.c
//
// Set SPI speed less then 10Mbps/s and more than 2*NRF_SetDataRate() ( - 2Mbps)
// Define NRF_CSN — Chip Select and NRF_CE — Chip Enable
//! Для обеспечения обратной совместимости с nRF2401A, nRF2402, nRF24E1 и nRF24E2 необходимо запретить Enhanced ShockBurst™, т.е. EN_AA = 0x00(NRF_Set_Auto_Ack(false))) и ARC = 0. При этом скорость передачи данных по радио должна быть установлена на 1Mbps или 250kbps.
#include "nrf24.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"
#include "semphr.h"
#include "delayus.h"
//!#include "spilib.h"
//!
#include "main.h"
#include "stm32f1xx_hal.h"
//include "dwtdelay.h"
//! Interrupt pin can be used

extern SPI_HandleTypeDef NRF24_SPI;
extern osSemaphoreId_t Spi2TxBinarySemHandle;


/*
// Definitions
//-----
//NRF
#define _RF_NRF24L01_					1
//#define _RF_NRF24L01P_				1
//#define _RF_SI24R1_						1
#define _SPI_NOTIFICATION_		1
//#define _SPI_SEMAPHORE_				1
#define NRF24_SPI 						hspi2
#define NRF_CS_Pin 					GPIO_PIN_12
#define NRF_CS_GPIO_Port 		GPIOB
#define NRF_CE_Pin 						GPIO_PIN_4
#define NRF_CE_GPIO_Port 			GPIOB
//!
#define NRF_INTERRUPT_ON			0x1
//-----
*/
//example use
/*
nrf_header.uc_type = RF_NRF24L01;
nrf_header.payload_size = 0;
nrf_header.dynamic_payloads_enabled = false;
nrf_header.pipe0_reading_address[5] = {0,};
nrf_header.addr_width = 0;
nrf_header.txDelay = 0;
*/
/*
//Config SPI
8 bits, MSB First, Boud Rate 4 MBits/s, Clock polarity LOW, Clock Phase 1 Edge, NVIC Global Interrupt.
*/

///!-#define DWT_CONTROL *(volatile unsigned long *)0xE0001000
///!-#define SCB_DEMCR   *(volatile unsigned long *)0xE000EDFC

//! global variables
//bool uc_type; 														// False for RF24L01 and true for RF24L01P 
//uint8_t payload_size = 0; 								// Fixed size of payloads 
//bool dynamic_payloads_enabled; 						// Whether dynamic payloads are enabled. 
//uint8_t pipe0_reading_address[5] = {0,}; 	// Last address set on pipe 0 for reading. 
//uint8_t addr_width = 0; 									// The address width to use - 3,4 or 5 bytes. 
//uint8_t txDelay = 0;



/*///!-
void DWT_Init(void)
{
  SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // allow counter
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // start counter
}

void delay_us(uint32_t us) // DelayMicro
{
    uint32_t us_count_tic =  us * (SystemCoreClock / 1000000);
    DWT->CYCCNT = 0U; // reset counter
    while(DWT->CYCCNT < us_count_tic);
}
*////!-
/*
void Set_CS_1(uint8_t level)
{
	HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, level);
	//delay_us(5);
}
*/
/*
void Set_CE_1(uint8_t level)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, level);
}
*/

// Function reads register reg and return data.
uint8_t NRF_Read_Reg( uint8_t reg ) {	
	reg = R_REGISTER | (REGISTER_MASK & reg);
	uint8_t data = 0;

	Set_CS_1(LOW);
	// Read data for command.
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, &reg, &data, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	// Read data from register.
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, (uint8_t*)0xff, &data, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	Set_CS_1(HIGH);
	
	return data;
}

// Function transmits data from buf to registers scince reg and returns command answer.
uint8_t NRF_Write_Buf( uint8_t reg, uint8_t* buf, uint8_t len ) {
	
	uint8_t command_answer = 0;
	reg = W_REGISTER | ( REGISTER_MASK & reg );
	
	Set_CS_1(LOW);
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, &reg, &command_answer, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	HAL_SPI_Transmit_IT( &NRF24_SPI, buf, len );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	Set_CS_1(HIGH);

	return command_answer;
}

// Function transmits data to register reg and returns command answer.
HAL_StatusTypeDef NRF_Write_Reg(uint8_t reg, uint8_t data)
{
	uint8_t command_answer = 0;
	reg = W_REGISTER | (REGISTER_MASK & reg);

	Set_CS_1( HIGH );
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, &reg, &command_answer, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	HAL_SPI_Transmit_IT( &NRF24_SPI, &data, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	Set_CS_1( LOW );
	
	return command_answer;
}
// Function writes payload.
uint8_t NRF_Write_Payload( nrf24_header_t* nrf_header, uint8_t addr, uint8_t* buffer, uint8_t len ) {
	//! HAL_StatusTypeDef errorcode = HAL_ERROR;
	len = MATH_MIN( len, nrf_header->payload_size );
	uint8_t command_answer = 0;
	
	Set_CS_1(LOW);
	// Transmit command and reseive answer.
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, &addr, &command_answer, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	// Fill payload by buffer.	
	HAL_SPI_Transmit_IT( &NRF24_SPI, buffer, len );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	// If dynamic payload enabled, fill the end of zeros.
	if( nrf_header->dynamic_payloads_enabled ) {
		uint8_t empt[32] = {0};
		
		len = ( nrf_header->payload_size ) - len;
		HAL_SPI_Transmit_IT( &NRF24_SPI, empt, len );
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	}
	Set_CS_1(HIGH);
	
	return command_answer;
}

// Function reads payload.
uint8_t NRF_Read_Payload( nrf24_header_t* nrf_header, uint8_t* buf, uint8_t len) {
	HAL_StatusTypeDef errorcode = HAL_OK;
	uint8_t addr = R_RX_PAYLOAD;
	len = MATH_MIN( len, nrf_header->payload_size );
	
	for( uint8_t i = 10; !(errorcode & i); i--) {
		Set_CS_1(LOW);
		// Transmit command and reseive answer.
		errorcode |= HAL_SPI_Transmit_IT( &NRF24_SPI, &addr, 1 );
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		// Fill payload by buffer.
		errorcode |= HAL_SPI_Receive_IT( &NRF24_SPI, buf, len );
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		// If dynamic payload enabled, fill the end of zeros.
		if( nrf_header->dynamic_payloads_enabled ) {
			uint8_t empt[32] = {0};
			
			len = ( nrf_header->payload_size ) - len;
			errorcode |= HAL_SPI_Receive_IT( &NRF24_SPI, empt, len );
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		}
		Set_CS_1(HIGH);
	}
	
	return (uint8_t)errorcode;
}

uint8_t NRF_Flush_RX(void) {
	
	return NRF_Command_TXRX( FLUSH_RX );
}

uint8_t NRF_Flush_TX( void ) {
	
	return NRF_Command_TXRX( FLUSH_TX );
}

uint8_t NRF_Command_TXRX( uint8_t command ) {
	uint8_t command_answer = 0;
	
	Set_CS_1(LOW);
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, &command, &command_answer, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	Set_CS_1(HIGH);
	
	return command_answer;
}

uint8_t NRF_Get_Status( void ) {
	
	return NRF_Command_TXRX( NOP );
}

void NRF_Set_Channel( uint8_t channel ) {
	
	NRF_Write_Reg( RF_CH, channel );
}

uint8_t NRF_Get_Channel() {
	
	return NRF_Read_Reg( RF_CH );
}

void NRF_Set_Payload_Size( nrf24_header_t* nrf_header, uint8_t size ) {
	
	nrf_header->payload_size = MATH_MIN( size, 32 );
}

uint8_t NRF_GetPayloadSize( nrf24_header_t* nrf_header ) {
	
	return nrf_header->payload_size;
}

uint8_t NRF_Init( nrf24_header_t* nrf_header ) {
	
	uint8_t setup = 0;

	Set_CE_1(LOW);
	Set_CS_1(HIGH);
	HAL_Delay(5);

	uint8_t CurrentRegister = 0 << MASK_RX_DR |		// Mask interrupt caused by RX_DR.
														0 << MASK_TX_DS |		// Mask interrupt caused by TX_DS.
														0 << MASK_MAX_RT |	// Mask interrupt caused by MAX_RT.
														1 << EN_CRC |				// Enable CRC.
														1 << CRCO |					// CRC encoding scheme.
														0 << PWR_UP |				// Power control.
														0 << PRIM_RX ;			// RX/TX control.
	// Reset NRF_CONFIG and enable 16-bit CRC.
	NRF_Write_Reg( NRF_CONFIG, CurrentRegister ); 
	CurrentRegister = ( 5 & 0xf ) << ARD | 				// (X+1)*250 us delay of retries.
										( 15& 0xf ) << ARC ; 				// X - count of retries. 
	// Set retries.
	NRF_Write_Reg( SETUP_RETR, CurrentRegister ); 
	// Set Power
	NRF_Set_PALevel(RF24_PA_0DBM);	
	//! uc_type = NRF_Check_Uc();
	setup = NRF_Read_Reg(RF_SETUP);
	// Slowest speed supported by all types of uc.
	NRF_SetDataRate(RF24_1MBPS); 
	// Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
	NRF_Toggle_Activate();
	NRF_Write_Reg(FEATURE, 0);
	NRF_Write_Reg(DYNPD, 0);
	//!
	nrf_header->dynamic_payloads_enabled = false;
	// Reset current status. Write 1 to clear bits. 
	// Notice reset and flush is the last thing we do.
	NRF_Write_Reg( NRF_STATUS, 	(1 << RX_DR) | 	//
															(1 << TX_DS) | 	//
															(1 << MAX_RT) 	//
	);
	NRF_Set_Channel(76);
	NRF_Flush_RX();
	NRF_Flush_TX();
	//Power up by default when begin() is called
	NRF_Power_Up(); 
	NRF_Write_Reg( NRF_CONFIG, ( NRF_Read_Reg( NRF_CONFIG ) ) & ~(1 << PRIM_RX) );
	
	return (setup != 0 && setup != 0xff);
}

// Checks connection with target.
bool NRF_IsChipConnected() {
	uint8_t setup = NRF_Read_Reg( SETUP_AW );

	if( setup >= 1 && setup <= 3 ) {
		return true;
	}
	return false;
}

void NRF_Start_Listening( nrf24_header_t* nrf_header ) {
	
	NRF_Power_Up();
	NRF_Write_Reg( NRF_CONFIG, NRF_Read_Reg( NRF_CONFIG ) | (1 << PRIM_RX) );
	NRF_Write_Reg(NRF_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
	Set_CE_1(HIGH);
	// Restore the pipe0 adddress, if exists
	if(nrf_header->pipe0_reading_address[0] > 0) {
		NRF_Write_Buf( RX_ADDR_P0, nrf_header->pipe0_reading_address, nrf_header->addr_width );
	} else {
		NRF_Close_Read_Pipe( nrf_header, 0 );
	}

	if( NRF_Read_Reg( FEATURE ) & (1 << EN_ACK_PAY) ) {
		NRF_Flush_TX();
	}
}

void NRF_Stop_Listening( nrf24_header_t* nrf_header ) {
	
	Set_CE_1(LOW);
	delay_us(nrf_header->txDelay);

	if(NRF_Read_Reg(FEATURE) & (1 << EN_ACK_PAY)) {
		delay_us(nrf_header->txDelay); //200
		NRF_Flush_TX();
	}

	NRF_Write_Reg( NRF_CONFIG, ( NRF_Read_Reg( NRF_CONFIG ) ) & ~(1 << PRIM_RX) );
	// Enable RX on pipe0
	NRF_Write_Reg( EN_RXADDR, NRF_Read_Reg( EN_RXADDR ) | ( 1 << nrf_header->child_pipe_enable[0] ) ); 
}

// Power down. chip go to Standby I.
void NRF_Power_Down(void) {
	// If CE was HIGH.
	Set_CE_1(LOW);
	uint8_t reg = NRF_Read_Reg( NRF_CONFIG );
	NRF_Write_Reg( NRF_CONFIG, reg & ~(1 << PWR_UP) );
}

// Power up chip.
void NRF_Power_Up( void ) {
	uint8_t cfg = NRF_Read_Reg( NRF_CONFIG );

	if( !( cfg & ( 1 << PWR_UP ) ) ) {
		NRF_Write_Reg( NRF_CONFIG, cfg | (1 << PWR_UP) );
		HAL_Delay(5);
	}
}

// Writes payload with multicast. Reads status. Clears the interrupt flags.
// Returns 0 if retransmit is off else returns 1.
bool NRF_WritePayloadAndClearIRQ( nrf24_header_t* nrf_header, void* buf, uint8_t len) {
	
	// Writes buf to payload with multicast
	uint8_t addr = W_TX_PAYLOAD_NO_ACK;// W_TX_PAYLOAD,	W_ACK_PAYLOAD
	NRF_Write_Payload( nrf_header, addr, (uint8_t*)buf, len ); 
	
	// Starts TX.
	Set_CE_1(HIGH);
	// Waits while Data_Sent and Retransmit from NRF will ready.
	// ACK is received and Maximum number of TX retransmits interrupt.
	while( !(NRF_Get_Status() & ((1 << TX_DS) | (1 << MAX_RT))) ) {
		HAL_Delay(5);
	}
	Set_CE_1(LOW);
		
	// Clears the interrupt flags.
	uint8_t status = NRF_Write_Reg( NRF_STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT) );
	
	// If Retransmit is off.
	if( status & (1 << MAX_RT) ) {
		NRF_Flush_TX(); 
		
		return 0;
	}
	return 1;
}

// Sets IRQ registers.
void NRF_Set_IRQ( bool tx, bool retx, bool rx ) {
	uint8_t reg = NRF_Read_Reg( NRF_CONFIG );
	
	// Clears the interrupt flags.
	reg &= ~( 1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR ); 
	// Sets the interrupt flags.
	reg |= retx << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR; 
	NRF_Write_Reg( NRF_CONFIG, reg );
}

uint8_t NRF_Get_Payload_Size( void ) {
	//HAL_StatusTypeDef    errorcode = HAL_OK;
	uint8_t result = 0; 
	uint8_t addr = R_RX_PL_WID;
	uint8_t addr2 = 0xff;
	
	Set_CS_1(LOW);
	// Transmits command for receive payload.
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, &addr, &result, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	// Receive payload.
	HAL_SPI_TransmitReceive_IT( &NRF24_SPI, &addr2, &result, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	Set_CS_1(HIGH);

	if(result > 32) {
		NRF_Flush_RX();
		HAL_Delay(2);
		// Something went wrong.
		return 0;
	}

	return result;
}

bool availableMy( void ) {
	
	return available( NULL );
}

bool available( uint8_t* pipe_num ) {
	
	if( !(NRF_Read_Reg( FIFO_STATUS ) & (1 << RX_EMPTY)) ) {
		// If the caller wants the pipe number, include that
		if( pipe_num ) {
			uint8_t status = NRF_Get_Status();
			*pipe_num = ( status >> RX_P_NO ) & 0x07;
		}
		return 1;
	}
	return 0;
}

// Reads payload and clears the interrupt flags.
void NRF_ReadPayloadAndClearIRQ( nrf24_header_t* nrf_header, void* buf, uint8_t len ) {
	
	NRF_Read_Payload( nrf_header, buf, len );
	NRF_Write_Reg( NRF_STATUS, (1 << RX_DR) | (1 << MAX_RT) | (1 << TX_DS) );
}

void NRF_Open_Writing_Pipe( nrf24_header_t* nrf_header, uint64_t value ) {
	
	NRF_Write_Buf( RX_ADDR_P0, (uint8_t*)&value, nrf_header->addr_width );
	NRF_Write_Buf( TX_ADDR, (uint8_t*)&value, nrf_header->addr_width );
	NRF_Write_Reg( RX_PW_P0, nrf_header->payload_size );
}

void NRF_Open_Reading_Pipe( nrf24_header_t* nrf_header, uint8_t child, uint64_t address ) {
	
	if( !child ){
		memcpy( nrf_header->pipe0_reading_address, &address, nrf_header->addr_width );
	}
	// For pipes 2-5, only writes the LSB.
	if( child < 2 ) {
		  NRF_Write_Buf( nrf_header->child_pipe[child], (uint8_t*)&address, nrf_header->addr_width );}
	else if( child <= 6 ) {
		NRF_Write_Reg( nrf_header->child_payload_size[child], nrf_header->payload_size );
		NRF_Write_Reg( EN_RXADDR, NRF_Read_Reg( EN_RXADDR ) | ( 1 << nrf_header->child_pipe_enable[child] ) );
	}
	else 
		NRF_Write_Buf( nrf_header->child_pipe[child], (uint8_t*)&address, 1 );
}

void NRF_Set_Addr_Width( nrf24_header_t* nrf_header, uint8_t a_width ) {
	
	if( a_width > 2 ) {
		NRF_Write_Reg( SETUP_AW, ( a_width - 2 ) % 4 );
		nrf_header->addr_width = a_width;
	} else {
		NRF_Write_Reg( SETUP_AW, 0 );
		nrf_header->addr_width = 2;
  }
}

void NRF_Close_Read_Pipe( nrf24_header_t* nrf_header, uint8_t pipe ) {
	
	NRF_Write_Reg( EN_RXADDR, NRF_Read_Reg( EN_RXADDR ) & ~( 1 << nrf_header->child_pipe_enable[pipe] ) );
}

// Activates/deactivates IRQ of payload. R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK.
void NRF_Toggle_Activate( void ) {
	uint8_t addr = ACTIVATE;
	
	Set_CS_1(LOW);
	HAL_SPI_Transmit_IT( &NRF24_SPI, &addr, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	HAL_SPI_Transmit_IT( &NRF24_SPI, (uint8_t*)0x73, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	Set_CS_1(HIGH);
}

// Turns ON dynamic payloads.
void NRF_ON_Dynamic_Payloads( void ) {
	
	NRF_Write_Reg( FEATURE, NRF_Read_Reg( FEATURE ) | EN_DPL 
																									//|	EN_ACK_PAY 
																									//|	EN_DYN_ACK 
								);
	NRF_Write_Reg( DYNPD, NRF_Read_Reg( DYNPD ) | DPL_P5  
																						|	DPL_P4 
																						|	DPL_P3  
																						|	DPL_P2  
																						|	DPL_P1   
																						|	DPL_P0
								);
}

void NRF_OFF_Dynamic_Payloads( void ) {
	
	NRF_Write_Reg( FEATURE, 0 );
	NRF_Write_Reg( DYNPD, 0 );
}

void NRF_Enable_Ack_Payload( nrf24_header_t* nrf_header ) {
	
	NRF_Write_Reg( FEATURE, NRF_Read_Reg( FEATURE ) | ( 1 << EN_ACK_PAY ) | ( EN_DPL ) );
	NRF_Write_Reg( DYNPD, NRF_Read_Reg( DYNPD ) | ( 1 << DPL_P1 ) | ( 1 << DPL_P0 ) );
	nrf_header->dynamic_payloads_enabled = true;
}

void NRF_Enable_Dynamic_Ack(void) {
	
    NRF_Write_Reg( FEATURE, NRF_Read_Reg( FEATURE ) | ( 1 << EN_DYN_ACK ) );
}

void NRF_Write_Ack_Payload( uint8_t pipe, uint8_t* buf, uint8_t len ) {
	len = MATH_MIN( len, 32 );
	uint8_t addr = W_ACK_PAYLOAD | ( pipe & 0x07 );
	
	Set_CS_1(LOW);
	HAL_SPI_Transmit_IT( &NRF24_SPI, &addr, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	HAL_SPI_Transmit_IT( &NRF24_SPI, buf, len );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	Set_CS_1(HIGH);
}

bool NRF_IsAckPayloadAvailable( void ) {
	
	return !( NRF_Read_Reg( FIFO_STATUS ) & ( 1 << RX_EMPTY ) );
}

rf24_type_uc_e NRF_UC_Variant( nrf24_header_t* nrf_header ) {
	
	return nrf_header->uc_type;
}

void NRF_Set_Auto_Ack( bool enable ) {
	
	if( enable )
		NRF_Write_Reg(EN_AA, 0x3F);
	else
		NRF_Write_Reg(EN_AA, 0);
}

void NRF_Set_Auto_AckPipe( uint8_t pipe, bool enable ) {
	
	if( pipe <= 6 ){
		uint8_t en_aa = NRF_Read_Reg( EN_AA );
		if( enable ) {
			en_aa |= ( 1 << pipe );
		} else {
			en_aa &= ~( 1 << pipe );
		}
		NRF_Write_Reg( EN_AA, en_aa );
	}
}

void NRF_Set_PALevel( uint8_t level ) {
	uint8_t rfsetup = NRF_Read_Reg(RF_SETUP) & 0xF8;

  if( level > 3 ) {
	  level = (RF24_PA_0DBM << 1) + 1;		// +1 to support the SI24R1 chip extra bit
  }
  else {
	  level = (level << 1) + 1;	
  }

  NRF_Write_Reg(RF_SETUP, rfsetup |= level);
}

uint8_t NRF_Get_PALevel( void ) {
	
	return ( NRF_Read_Reg( RF_SETUP ) & ( ( 1 << RF_PWR_LOW ) | ( 1 << RF_PWR_HIGH ) ) ) >> 1;
}

// Sets data rate and returns delay for transmit.
uint8_t NRF_SetDataRate( rf24_datarate_t speed ) {

	uint8_t setup = NRF_Read_Reg( RF_SETUP );
	// Set 3 and 5 bits to 0.
	setup &= ~( ( 1 << RF_DR_LOW ) | ( 1 << RF_DR_HIGH ) );
	uint8_t txDelay = RF_DELAY_1MBPS;
	if( speed == RF24_250KBPS ) {
		setup |= ( 1 << RF_DR_LOW );
		txDelay = RF_DELAY_250KBPS;
	}
	else {
		if( speed == RF24_2MBPS ) {
			setup |= ( 1 << RF_DR_HIGH );
			txDelay = RF_DELAY_2MBPS;
		}
	}
//!
	NRF_Write_Reg(RF_SETUP, setup);
	if( NRF_Read_Reg( RF_SETUP ) == setup ) {
		
		return txDelay;
	}
}

// Returns setted data rate.
rf24_datarate_t NRF_GetDataRate( void ) {
	
	uint8_t dr = ( NRF_Read_Reg( RF_SETUP ) ) & ( 1 << RF_DR_LOW | 1 << RF_DR_HIGH );

	if( ( dr >> RF_DR_LOW ) & 0x1 ) {
		
		return RF24_250KBPS;
	}
	if( ( dr >> RF_DR_HIGH ) & 0x1 )  {
		
		return RF24_2MBPS;
	}
	if ( ( dr >> RF_DR_HIGH ) & 0x1 )  {
		
		return RF24_1MBPS;
	}

		return RF24_NS;

}

void NRF_Set_CRC_Length( rf24_crclength_e length ) {
	
	uint8_t config = NRF_Read_Reg( NRF_CONFIG ) & ~( ( 1 << CRCO ) | ( 1 << EN_CRC ) );

	if( length == RF24_CRC_DISABLED ) {
		NRF_Disable_CRC();
	} 
	else if( length == RF24_CRC_8 ) {
		config |= ( 1 << EN_CRC );
	}
	else {
		config |= ( 1 << EN_CRC );
		config |= ( 1 << CRCO );
	}
	NRF_Write_Reg( NRF_CONFIG, config );
}

rf24_crclength_e NRF_Get_CRC_Length( void ) {
	rf24_crclength_e result = RF24_CRC_DISABLED;
	uint8_t config = NRF_Read_Reg( NRF_CONFIG ) & ( ( 1 << CRCO ) | ( 1 << EN_CRC ) );
	uint8_t AA = NRF_Read_Reg( EN_AA );

	if( config & ( 1 << EN_CRC ) || AA ) {
		if( config & ( 1 << CRCO ) )
		  result = RF24_CRC_16;
		else
		  result = RF24_CRC_8;
	}

	return result;
}

void NRF_Disable_CRC( void ) {
	uint8_t disable = NRF_Read_Reg(NRF_CONFIG) & ~(1 << EN_CRC);
	
	NRF_Write_Reg(NRF_CONFIG, disable);
}

void NRF_Set_Retries( uint8_t delay, uint8_t count ) {

	NRF_Write_Reg( SETUP_RETR, ( delay&0xf )<<ARD | ( count&0xf )<<ARC );
}


// Returns type of ucontroller.
rf24_type_uc_e NRF_Check_Uc( void ) {
	
	#ifdef _RF_NRF24L01P_
	return RF_NRF24L01P;
	#elif _RF_SI24R1_
	return RF_SI24R1;
	#elif _RF_NRF24L01_
	return RF_NRF24L01;
	#elif
	NRF_SetDataRate( RF24_250KBPS );
	if( ( NRF_Read_Reg( RF_SETUP ) >> RF_DR_LOW ) == 1 ) {
		
		#define _RF_NRF24L01P_	0x01
		return RF_NRF24L01P;
	}//else if(  ) {
		
		//#define _RF_SI24R1_	0x01
		//return RF_SI24R1;
	//} 
	else {
		
		#define _RF_NRF24L01_	0x01
		return RF_NRF24L01;
	}
	#endif
}

//! ControlTech
//New lib
//TxMode
void NRF_TxMode( uint8_t* id, uint8_t channel ) {
	// disable the chip before configuring the device
	Set_CE_1( LOW );
	//Set_CS_1( LOW );
	NRF_Write_Reg( RF_CH, channel );
	NRF_Write_Buf( TX_ADDR, id, 5);
	// Power uo device.
	uint8_t config = NRF_Read_Reg( NRF_CONFIG );
	// write 1 in the PWR_UP bit
	//config |= 1 << 1;
	// write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	//! config &= 0xF2;
	config &= 	1 << 7							// Reserved.
						| 1 << MASK_RX_DR			// Mask interrupt caused by RX_DR.
						| 1 << MASK_TX_DS			// Mask interrupt caused by TX_DS.
						| 1 << MASK_MAX_RT		// Mask interrupt caused by MAX_RT.
						| 0 << EN_CRC 				// Enable CRC.
						| 0 << CRCO						// CRC encoding scheme. '0' - 1 byte, '1' – 2 bytes.
						| 1 << PWR_UP					// 1: POWER UP, 0:POWER DOWN.
						| 0 << PRIM_RX;				// RX/TX control. 1: PRX, 0: PTX.
	NRF_Write_Reg( NRF_CONFIG, config );
	// Enable device.
	Set_CE_1( HIGH );
}


// transmit the data
uint8_t NRF24_Transmit ( uint8_t* data ) {
	uint8_t cmdtosend = W_TX_PAYLOAD;
	// select the device
	Set_CS_1( LOW );
	// payload command
	HAL_SPI_Transmit_IT( &NRF24_SPI, &cmdtosend, 1 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	// send the payload
	HAL_SPI_Transmit_IT(&NRF24_SPI, data, 32 );
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	// Unselect the device
	Set_CS_1( HIGH );
	HAL_Delay(5);
	//! HAL_Delay(1);
	uint8_t fifostatus = NRF_Read_Reg( FIFO_STATUS );
	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ( fifostatus & ( 1 << TX_EMPTY ) && ( !(fifostatus&( 1 << FIFO_FULL ) ) ) )  {
		//NRF_Flush_TX(); 
		// reset FIFO_STATUS
		//!-nrf24_reset (FIFO_STATUS);
		
		return fifostatus;
	}
	return fifostatus;
}

void nrf24_reset(uint8_t REG)
{
	if (REG == NRF_STATUS)
	{
		NRF_Write_Reg(NRF_STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		NRF_Write_Reg(FIFO_STATUS, 0x11);
	}

	else {
	NRF_Write_Reg(NRF_CONFIG, 0x08);
	NRF_Write_Reg(EN_AA, 0x3F);
	NRF_Write_Reg(EN_RXADDR, 0x03);
	NRF_Write_Reg(SETUP_AW, 0x03);
	NRF_Write_Reg(SETUP_RETR, 0x03);
	NRF_Write_Reg(RF_CH, 0x02);
	NRF_Write_Reg(RF_SETUP, 0x0E);
	NRF_Write_Reg(NRF_STATUS, 0x00);
	NRF_Write_Reg(OBSERVE_TX, 0x00);
	NRF_Write_Reg(CD, 0x00);
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF_Write_Buf(RX_ADDR_P0, rx_addr_p0_def, 5);
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	NRF_Write_Buf(RX_ADDR_P1, rx_addr_p1_def, 5);
	NRF_Write_Reg(RX_ADDR_P2, 0xC3);
	NRF_Write_Reg(RX_ADDR_P3, 0xC4);
	NRF_Write_Reg(RX_ADDR_P4, 0xC5);
	NRF_Write_Reg(RX_ADDR_P5, 0xC6);
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF_Write_Buf(TX_ADDR, tx_addr_def, 5);
	NRF_Write_Reg(RX_PW_P0, 0);
	NRF_Write_Reg(RX_PW_P1, 0);
	NRF_Write_Reg(RX_PW_P2, 0);
	NRF_Write_Reg(RX_PW_P3, 0);
	NRF_Write_Reg(RX_PW_P4, 0);
	NRF_Write_Reg(RX_PW_P5, 0);
	NRF_Write_Reg(FIFO_STATUS, 0x11);
	NRF_Write_Reg(DYNPD, 0);
	NRF_Write_Reg(FEATURE, 0);
	}
}

void NRF24_Init2 (void)
{
	// disable the chip before configuring the device
	//CE_Disable();
	// Enable device.
	Set_CE_1( LOW );

	// reset everything
	nrf24_reset (0);

	NRF_Write_Reg(NRF_CONFIG, 0);  // will be configured later

	NRF_Write_Reg(EN_AA, 0);  // No Auto ACK

	NRF_Write_Reg (EN_RXADDR, 0);  // Not Enabling any data pipe right now

	NRF_Write_Reg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address

	NRF_Write_Reg (SETUP_RETR, 0);   // No retransmission

	NRF_Write_Reg (RF_CH, 0);  // will be setup during Tx or RX

	NRF_Write_Reg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps

	// Enable the chip after configuring the device
	//CE_Enable();
	// Enable device.
	Set_CE_1( HIGH );
}
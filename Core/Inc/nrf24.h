//NRF24L1
#ifndef NRF24_H_
#define NRF24_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
//!#include "delayus.h"
	 //!
#include "stm32f1xx_hal.h"
//SPI
#define _SPI_1_LIB_						1
#define SPI1_CS_Pin 					NRF_CS_Pin
#define SPI1_CS_Port					NRF_CS_GPIO_Port
#define SPI1_CE_Pin 					NRF_CE_Pin
#define SPI1_CE_Port					NRF_CE_GPIO_Port	
//#define NRF24_SPI 						hspi2
//!#include "spilib.h"
	 
#ifdef SPI1_CS_Pin
	#define Set_CS_1( PinState )			HAL_GPIO_WritePin( SPI1_CS_Port, SPI1_CS_Pin, PinState );
#endif 
	 
#ifdef SPI1_CE_Pin
	#define Set_CE_1( PinState )			HAL_GPIO_WritePin( SPI1_CE_Port, SPI1_CE_Pin, PinState );
#endif 
	 
//#define Set_CS( PinState )			Set_CS_1( PinState );	 

#if( _SPI_SEMAPHORE_ == 1 )
	#define _END_SPI_TX()								xSemaphoreTake( Spi2TxBinarySemHandle, portMAX_DELAY );
#elif ( _SPI_NOTIFICATION_ == 1 )
	#define  _END_SPI_TX()							ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
#endif	// _SPI_SEMAPHORE_ _SPI_NOTIFICATION_
	 
#define MATH_MAX( a, b ) ( a > b ? a : b )
#define MATH_MIN( a, b ) ( a < b ? a : b )

/* Memory Map */
#define NRF_CONFIG  0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05			// Operating frequency. 2400 MHz + RF_CH (0..125)
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    	0x1C			// Enable dynamic payload length.
#define FEATURE	    0x1D

/* Bit Mnemonics */
#define HIGH 				1
#define LOW  				0
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0					// RX/TX control. 1: PRX, 0: PTX.
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6							// Power Amplifier. From -18dbm to 0.
#define RX_DR       6
#define TX_DS       5							// Data Sent TX FIFO interrupt. Write 1 to clear bit.
#define MAX_RT      4							// Maximum number of TX retransmits interrupt. Write 1 to clear bit.
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6							// TX payload reuse. Commands REUSE_TX_PL, W_TX_PAYLOAD and FLUSH TX. For a PTX device.
#define FIFO_FULL   5							// TX FIFO.
#define TX_EMPTY    4							 
#define RX_FULL     1							
#define RX_EMPTY    0							
#define DPL_P5	    0x1 << 5			// Enable dyn. payload length data pipe 5.
#define DPL_P4	    0x1 << 4			// Enable dyn. payload length data pipe 4.
#define DPL_P3	    0x1 << 3			// Enable dyn. payload length data pipe 3.
#define DPL_P2	    0x1 << 2			// Enable dyn. payload length data pipe 2.
#define DPL_P1	    0x1 << 1			// Enable dyn. payload length data pipe 1.
#define DPL_P0	    0x1 << 0			// Enable dyn. payload length data pipe 0.
#define EN_DPL	    0x1 << 2			// Enables Dynamic Payload Length.
#define EN_ACK_PAY  0x1 << 1			// Enables Payload with ACK.
#define EN_DYN_ACK  0x1 << 0			// Enables the W_TX_PAYLOAD_NOACK command.

/* Instruction Mnemonics */
#define R_REGISTER    0x00				// Read registers of command and state.
#define W_REGISTER    0x20				// Read registers of command and state for PowerDown and Standby mods.
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61				// Read RX payload. 0 - 32 bytes.
// TX_PLD register.
#define W_ACK_PAYLOAD 				0xA8		// Enable in PRX mode.
#define W_TX_PAYLOAD  				0xA0		// Enable in PTX mode, without multicast.
#define W_TX_PAYLOAD_NO_ACK  	0xB0		// P model memory Map. Enable in PTX mode, with multicast.
#define RPD         					0x09		// Received Power Detector. P model memory Map. From -64 dBm.
// 
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

// Non-P omissions
#define LNA_HCURR   	0





// P model bit
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

///////////////////////////////////////////////////////////////////////////////
typedef enum
{
	RF24_PA_M18DBM = 0,
	RF24_PA_M12DBM,
	RF24_PA_M6DBM,
	RF24_PA_0DBM,
	RF24_PA_ERROR
} rf24_pa_dbm_e;

typedef enum
{
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS,
	RF24_NS
} rf24_datarate_t;

typedef enum
{
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
} rf24_crclength_e;

typedef enum
{
	RF_DELAY_NS = 			0,
	RF_DELAY_250KBPS = 	155,
	RF_DELAY_1MBPS = 		85,
	RF_DELAY_2MBPS = 		65,
} rf24_delay_e;

typedef enum
{
	RF_NRF24L01 = 0,
	RF_NRF24L01P,
	RF_SI24R1
} rf24_type_uc_e;

typedef struct { 
rf24_type_uc_e		uc_type;
uint8_t 					payload_size;
bool 							dynamic_payloads_enabled; 	// Whether dynamic payloads are enabled. 
uint8_t 					pipe0_reading_address[5]; 	// Last address set on pipe 0 for reading. 
uint8_t 					addr_width; 								// The address width to use - 3,4 or 5 bytes. 
uint8_t 					txDelay;
uint8_t 					child_pipe_enable[6];
uint8_t						child_pipe[6];
uint8_t						child_payload_size[6];
} nrf24_header_t;

uint8_t NRF_Init( nrf24_header_t* nrf_header );
bool NRF_IsChipConnected();
void NRF_Start_Listening( nrf24_header_t* nrf_header );
void NRF_Stop_Listening( nrf24_header_t* nrf_header );
bool availableMy(void);
void NRF_ReadPayloadAndClearIRQ( nrf24_header_t* nrf_header, void* buf, uint8_t len);
bool NRF_WritePayloadAndClearIRQ( nrf24_header_t* nrf_header, void* buf, uint8_t len);
bool available(uint8_t* pipe_num);
uint8_t NRF_Command_TXRX(uint8_t cmd);
void NRF_Power_Down(void);
void NRF_Power_Up(void);
void NRF_Write_Ack_Payload( uint8_t pipe, uint8_t* buf, uint8_t len );
bool NRF_IsAckPayloadAvailable(void);
uint8_t whatHappened();
uint8_t NRF_Flush_TX( void );
void NRF_Close_Read_Pipe( nrf24_header_t* nrf_header, uint8_t pipe);
void NRF_Set_Addr_Width( nrf24_header_t* nrf_header, uint8_t a_width);
void NRF_Set_Retries(uint8_t delay, uint8_t count);
void NRF_Set_Channel(uint8_t channel);
uint8_t NRF_Get_Channel(void);
void NRF_Set_Payload_Size( nrf24_header_t* nrf_header, uint8_t size);
uint8_t NRF_GetPayloadSize( nrf24_header_t* nrf_header );
uint8_t NRF_Get_Payload_Size(void);
void NRF_Enable_Ack_Payload( nrf24_header_t* nrf_header );
void NRF_ON_Dynamic_Payloads(void);
void NRF_OFF_Dynamic_Payloads(void);
void NRF_Enable_Dynamic_Ack();
rf24_type_uc_e NRF_UC_Variant( nrf24_header_t* nrf_header );
void NRF_Set_Auto_Ack(bool enable);
void NRF_Set_Auto_AckPipe(uint8_t pipe, bool enable);
void NRF_Set_PALevel(uint8_t level);
uint8_t NRF_Get_PALevel(void);
HAL_StatusTypeDef NRF_SetDataRate(rf24_datarate_t speed);
rf24_datarate_t NRF_GetDataRate(void);
void NRF_Set_CRC_Length(rf24_crclength_e length);
rf24_crclength_e NRF_Get_CRC_Length(void);
void NRF_Disable_CRC(void);
void NRF_Set_IRQ(bool tx_ok,bool tx_fail,bool rx_ready);
void NRF_Open_Reading_Pipe( nrf24_header_t* nrf_header, uint8_t child, uint64_t address);
void NRF_Open_Writing_Pipe( nrf24_header_t* nrf_header, uint64_t value);
uint8_t NRF_Flush_RX(void);
uint8_t NRF_Read_Reg( uint8_t addr );
uint8_t NRF_Write_Buf( uint8_t reg, uint8_t* buf, uint8_t len );
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value);
uint8_t write_payload( nrf24_header_t* nrf_header, const void* buf, uint8_t data_len, const uint8_t writeType);
uint8_t NRF_Read_Payload( nrf24_header_t* nrf_header, uint8_t* buf, uint8_t len);
uint8_t NRF_Get_Status(void);
void NRF_Toggle_Activate(void);
rf24_type_uc_e NRF_Check_Uc( void );
void NRF_TxMode( uint8_t* id, uint8_t channel );
uint8_t NRF24_Transmit (uint8_t *data);
void nrf24_reset(uint8_t REG);
void NRF24_Init2 (void);

/////////////////////////////////////////////////////////////////////////////
//! void DWT_Init(void);
//void delay_us(uint32_t us);
//void NRF24_ini(void);

#ifdef __cplusplus
}
#endif

#endif /* NRF24H_ */

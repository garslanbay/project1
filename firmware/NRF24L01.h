/*
 * MF RC522 Default pinout
 * 
 * 		MFRC522		STM32F4XX	DESCRIPTION
 *		CS (SDA)	PD2			Chip select for SPI
 *		SCK		PB3			Serial Clock for SPI
 *		MISO		PB4			Master In Slave Out for SPI
 *		MOSI		PB5			Master Out Slave In for SPI
 *		GND		GND			Ground
 *		VCC		3.3V			3.3V power
 *		RST		3.3V			Reset pin
 */		

#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_spi.h"

//-----------------------------------------------------------
#ifndef M0_RC522_H
#define M0_RC522_H 100

#define MFRC522_SPI 			    SPI1

//-----------------------------------------------------------
#ifndef   MFRC522_CS_PIN 						// Default CS pin used 
  #define MFRC522_CS_RCC					  RCC_AHBPeriph_GPIOA
  #define MFRC522_CS_PORT					  GPIOA
  #define MFRC522_CS_PIN					  GPIO_Pin_4
#endif

//-----------Status enumeration Used with most functions------
typedef enum 
{
	MI_OK = 0,
	MI_NOTAGERR,
	MI_ERR
} M0_RC522_Status_t;
//------------------------------------------------------------

#define MFRC522_CS_LOW					GPIO_ResetBits(MFRC522_CS_PORT, MFRC522_CS_PIN);
#define MFRC522_CS_HIGH					GPIO_SetBits(MFRC522_CS_PORT, MFRC522_CS_PIN);//MFRC522_CS_PORT->BSRRL = MFRC522_CS_PIN;

/* MFRC522 Commands */
#define READ			  					0x00 
#define WRITE									0x20   
#define R_RX_PAYLOAD					0x61   
#define W_TX_PAYLOAD					0xA0   //Transmit data
#define FLUSH_TX							0xE1   //Transmit and receive data,
#define FLUSH_RX							0xE2   //Reset
#define REUSE_TX_PL						0xE3  //CRC Calculate
#define ACTIVATE							0x50  // find the antenna area does not enter hibernation
#define NOP										0xFF// find all the cards antenna area
#define R_RX_PL_WID						0x60 // anti-collision


#define RF_Max_addr				0xFFFF

typedef union data_adres
{
	uint16_t adress;
	struct adrs
	{
		uint8_t adres_l;
		uint8_t adres_h;
	}adres;

}data11;


//Config adress 0 
//--------------------------------------
#define DIS_IRQ_MASK_RX_DR 		0x40
#define EN_IRQ_MASK_RX_DR 		0x00
//--------------------------------------
#define DIS_IRQ_MASK_TX_DS		0x20
#define EN_IRQ_MASK_TX_DS			0x00
//--------------------------------------
#define DIS_IRQ_MASK_MAX_RT		0x10
#define EN_IRQ_MASK_MAX_RT		0x00
//--------------------------------------
#define EN_CRC								0x08
#define DIS_CRC								0x00
//--------------------------------------
#define CRCO_1BYTE						0x00
#define CRCO_2BYTE						0x04
//--------------------------------------
#define PWR_UP								0x02
#define PWR_DOWN							0x00
//--------------------------------------
#define PRIM_RX								0x01
#define PRIM_TX								0x00
//--------------------------------------
//*******************************************//
//config adress 1 EN_AA Enhanced ShockBurst™
//--------------------------------------
#define ENAA_P0								0x01
#define DISAA_P0							0x00
//--------------------------------------
#define ENAA_P1								0x02
#define DISAA_P1							0x00
//--------------------------------------
#define ENAA_P2								0x04
#define DISAA_P2							0x00
//--------------------------------------
#define ENAA_P3								0x08
#define DISAA_P3							0x00
//--------------------------------------
#define ENAA_P4								0x10
#define DISAA_P5							0x00
//--------------------------------------
#define ENAA_P5								0x20
#define DISAA_P5							0x00
//--------------------------------------

//************************************//
//config adress 2  EN_RXADDR

//--------------------------------------
#define ERX_P0								0x00
#define DISRX_P0							0x01
//--------------------------------------
#define ERX_P1								0x00
#define DISRX_P1							0x02
//--------------------------------------
#define ERX_P2								0x00
#define DISRX_P2							0x04
//--------------------------------------
#define ERX_P3								0x00
#define DISRX_P3							0x08
//--------------------------------------
#define ERX_P4								0x00
#define DISRX_P4							0x10
//--------------------------------------
#define ERX_P5								0x00
#define DISRX_P5							0x20
//--------------------------------------

//************************************//

//config adress 3 SETUP_AW (Setup of Address Widths)

#define AW_3BYTE							0x01
//--------------------------------------
#define AW_4BYTE							0x02
//--------------------------------------
#define AW_5BYTE							0x03
//--------------------------------------

//************************************//

//config adress 4 SETUP_RETR (Setup of Automatic Retransmission)

#define ARD(delay,count)			(delay) || count)					


//------------------------Initialize MFRC522 RFID reader-----------
extern void RF_Init(void);


/**
 * Private functions
 */
extern void RF_InitPins(void);
extern void RF_WriteRegister(data11* addr, uint8_t val);
extern uint8_t RF_ReadRegister(data11* addr);
extern uint16_t RF_Write_Page(uint16_t blockAddr, uint8_t* writeData,uint16_t cnt ) ;

extern void RF_Write_Enable(void);
extern void RF_Write_Disable(void);
static uint8_t RF_SendByte(uint8_t byte);
extern uint8_t read_status(void);
extern void write_status(void);
extern void delay_cycles( uint8_t a);
extern void mySPI_SendData( uint8_t data);
extern uint8_t mySPI_GetData(void);

extern uint8_t RF_ReadRegister2(uint8_t addr);
extern void RF_WriteRegister2(uint8_t addr, uint8_t val);

extern uint16_t	RF_blok_oku(uint16_t adress,uint8_t * data , uint8_t size);

uint8_t M0_SPI_Send(uint8_t data);

#endif


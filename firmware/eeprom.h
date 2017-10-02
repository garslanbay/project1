#ifndef __EEPROM_H
#define __EEPROM_H



/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_spi.h"


#define Read		0x03
#define Write		0x02
#define WR_DIS	0x04
#define WR_EN		0x06
#define RDSR		0x05		// read status register
#define WRSR		0x01


#define cs(a)		GPIO_WriteBit(GPIOA,GPIO_Pin_8,a);
#define clk(a)	GPIO_WriteBit(GPIOA,GPIO_Pin_5,a);
#define out(a)	GPIO_WriteBit(GPIOA,GPIO_Pin_7,a);
#define in			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)

extern void eeprom_yaz(uint16_t adress , uint8_t data);
extern uint8_t	eeprom_blok_oku(uint16_t adress,uint8_t * data , uint8_t size);
void WREN(void);
void WRDIS(void);
extern uint8_t read_stat(void);
void write_stat(uint8_t stat);
extern uint16_t eeprom_writepage(uint16_t adress , uint8_t* data , uint8_t cnt);
extern uint8_t send_byte (uint8_t Data);
void delay_cycles( uint8_t a);

uint8_t spi_oku(void);
void spi_yaz(uint8_t _byte);

void mySPI_SendData(uint8_t data);
uint8_t mySPI_GetData(void);


#endif /* __STM32F0XX_IWDG_H */
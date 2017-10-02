#include "eeprom.h"

uint8_t temp;

void WREN(void)
{
	cs(0);
	
	delay_cycles( 1 ); 
	
	//spi_yaz(WR_EN);

	mySPI_SendData(WR_EN);

	delay_cycles( 1 ); 
	
	cs(1);
}

void WRDIS(void)
{
	
	cs(0);
	
	delay_cycles( 1 ); 

	//spi_yaz(WR_DIS);
	mySPI_SendData(WR_DIS);
	
	delay_cycles( 1 ); 

	cs(1);
	
	
}

void eeprom_yaz(uint16_t adress , uint8_t data)
{
	WREN();

	
	cs(0);
	
	delay_cycles( 1 );
	
	//spi_yaz(Write);
	//spi_yaz((uint8_t)(adress>>8));
	//spi_yaz((uint8_t)(adress));
	//spi_yaz(data);
	mySPI_SendData(Write);

	mySPI_SendData((uint8_t)(adress>>8));

	mySPI_SendData((uint8_t)(adress));
	
	mySPI_SendData(data);
	
	delay_cycles( 1 );

	cs(1);

	
	WRDIS();
	
}

uint8_t	eeprom_oku(uint16_t adress)
{
	cs(0);
	
	
	delay_cycles( 1 );
	
	//spi_yaz(Read);
	//spi_yaz((uint8_t)(adress>>8));
	//spi_yaz((uint8_t)(adress));
	//temp=spi_oku();
	
	mySPI_SendData(Read);

	mySPI_SendData((uint8_t)(adress>>8));
	
	mySPI_SendData((uint8_t)(adress));
	
	temp=mySPI_GetData();

	delay_cycles( 1 );
	cs(1);
	
	return temp;
}

uint8_t read_stat(void)
{
	uint8_t temp1;
	
	cs(0);
	delay_cycles( 1 );
	//spi_yaz(RDSR);
	//temp=spi_oku();
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_SendData8(SPI1,RDSR);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	
	temp1=SPI_ReceiveData8(SPI1);
	
	delay_cycles( 1 );
	cs(1);
	
	return temp1;
}

void write_stat(uint8_t stat)
{
	
	cs(0);
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	//SPI_SendData8(SPI1,WRSR);
	//while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	//SPI_SendData8(SPI1,stat);
	cs(1);
	
	
}

uint16_t eeprom_writepage(uint16_t adress , uint8_t* data , uint8_t cnt)
{
	
	WREN();
	
	delay_cycles( 1 );
	
	cs(0);
	delay_cycles( 1 );
	
	mySPI_SendData(Write);

	mySPI_SendData((uint8_t)(adress>>8));

	mySPI_SendData((uint8_t)(adress));
	
	while(cnt--)
	{
		mySPI_SendData(*data);
		data++;
		adress++;
	}
	
	delay_cycles( 1 );
	
	cs(1);
	delay_cycles( 1 );
	
	WRDIS();
	
	return adress;
	
}

void spi_yaz(uint8_t _byte)
{
	uint8_t iii;

	for(iii=0;iii<8;iii++)
	{	
		if(_byte&0x80)
		{
			out(1);
		}
		else
		{
			out(0);
		}

		clk(1);				//CLK=1
		delay_cycles( 1 ); 
		delay_cycles( 1 );
		delay_cycles( 1 );
		delay_cycles( 1 ); 
	
		clk(0);				//CLK=0
		delay_cycles( 1 ); 
		delay_cycles( 1 );
		delay_cycles( 1 );
		delay_cycles( 1 ); 


		
		_byte<<=1;				//BYTE DEGISKENiNi 1 BiT SOLA KAYDIR
	}
}

uint8_t spi_oku(void)
{
	uint8_t iii,_byte=0;

	for(iii=0;iii<8;iii++)
	{
		
		_byte<<=1;

		clk(1);	

		delay_cycles( 1 ); 
		delay_cycles( 1 );
		delay_cycles( 1 );
		delay_cycles( 1 ); 


		if(in==SET)
		{
			_byte|=1;
		}

		clk(0);
		
		delay_cycles( 1 ); 
		delay_cycles( 1 );
		delay_cycles( 1 );
		delay_cycles( 1 ); 

	}

	return  _byte;
}

void delay_cycles( uint8_t a)
{
	uint8_t bb=255;
	while(bb--);

}

void mySPI_SendData( uint8_t data){
 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_SendData8(SPI1, data);
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	
	SPI_ReceiveData8(SPI1);
 
}

uint8_t mySPI_GetData(void)
{
 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_SendData8(SPI1, 0x00); //Dummy byte to generate clock
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
 
	return  SPI_ReceiveData8(SPI1);
}

uint8_t	eeprom_blok_oku(uint16_t adress,uint8_t * data , uint8_t size)
{
	
	uint8_t sayac=0;

	
	for(sayac=0;sayac<size;sayac++)
	{
	
		*data=eeprom_oku(adress);
		data++;
		adress++;
		
	}

	
	return adress;
}

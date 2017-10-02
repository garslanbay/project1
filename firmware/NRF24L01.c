#include <NRF24L01.h>

void RF_Init(void) 
{
	
	SPI_InitTypeDef SPI_InitStruct;
	
	RF_InitPins();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	//--------------------------------------------------
	SPI_I2S_DeInit (MFRC522_SPI);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial=7;
	SPI_Init(SPI1,&SPI_InitStruct);
	
	SPI_RxFIFOThresholdConfig(SPI1,SPI_RxFIFOThreshold_QF); 
	SPI_Cmd(SPI1,ENABLE);
	//SPI_SSOutputCmd(MFRC522_SPI, ENABLE);
//-----------------------------------------------------
}
void RF_InitPins(void) 
{
	GPIO_InitTypeDef SPI_GPIOB_Struct;
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
//-------------------------------------------------------------

	SPI_GPIOB_Struct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_6; //clk and mosi 
	SPI_GPIOB_Struct.GPIO_Mode = GPIO_Mode_AF;
	SPI_GPIOB_Struct.GPIO_OType = GPIO_OType_PP;
	SPI_GPIOB_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	SPI_GPIOB_Struct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &SPI_GPIOB_Struct);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5,  GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6,  GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7,  GPIO_AF_0);
	
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOAEN, ENABLE);	//Enable clock

	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	//CS pin
	GPIO_InitStruct.GPIO_Pin = MFRC522_CS_PIN;
	GPIO_Init(MFRC522_CS_PORT, &GPIO_InitStruct);	
	MFRC522_CS_HIGH; //CS- UnSelect
	//MFRC522_CS_LOW;//CS low
}

void RF_WriteRegister(data11* addr, uint8_t val)
 {
		RF_Write_Enable();
	 
		MFRC522_CS_LOW;//CS low
	 
		delay_cycles( 1 );
	
	//spi_yaz(Write);
	//spi_yaz((uint8_t)(adress>>8));
	//spi_yaz((uint8_t)(adress));
	//spi_yaz(data);
		mySPI_SendData(WRITE);

		mySPI_SendData(addr->adres.adres_h);

		mySPI_SendData(addr->adres.adres_l);
	
		mySPI_SendData(val);
	
		delay_cycles( 1 );
	 
		MFRC522_CS_HIGH;//CS high
	 
	 
	 
		RF_Write_Disable();


 }

uint8_t RF_ReadRegister(data11* addr)
 {
	uint8_t val;
	 
		MFRC522_CS_LOW; //CS low
	 
		delay_cycles( 1 );
	
	//spi_yaz(Read);
	//spi_yaz((uint8_t)(adress>>8));
	//spi_yaz((uint8_t)(adress));
	//temp=spi_oku();
	
		mySPI_SendData(READ);

		mySPI_SendData(addr->adres.adres_h);

		mySPI_SendData(addr->adres.adres_l);
	
		val=mySPI_GetData();

		delay_cycles( 1 );
	 
		MFRC522_CS_HIGH;//CS high
		return val;	
 }
 
 uint8_t RF_ReadRegister2(uint8_t addr)
 {
 uint8_t val;
	 
		MFRC522_CS_LOW; //CS low
	 
		delay_cycles( 1 );
	
	//spi_yaz(Read);
	//spi_yaz((uint8_t)(adress>>8));
	//spi_yaz((uint8_t)(adress));
	//temp=spi_oku();
	
		mySPI_SendData(addr);
	 delay_cycles( 1 );
	
		val=mySPI_GetData();

		delay_cycles( 1 );
	 
		MFRC522_CS_HIGH;//CS high
		return val;	
 
 
 }




//----------------------------------------------------------------------
void RF_WriteRegister2(uint8_t addr, uint8_t val)
 {
		RF_Write_Enable();
	 
		MFRC522_CS_LOW;//CS low
	 
		delay_cycles( 1 );
	
		mySPI_SendData(addr);
		delay_cycles( 1 );
		mySPI_SendData(val);
	
		delay_cycles( 1 );
	 
		MFRC522_CS_HIGH;//CS high
	 
	 
	 
		RF_Write_Disable();


 }
uint16_t RF_Write_Page(uint16_t blockAddr, uint8_t* writeData,uint16_t cnt ) 
 {
		data11 temp;
	 
		temp.adress=blockAddr;
	 
		RF_Write_Enable();
	
		delay_cycles( 1 );
	
		MFRC522_CS_LOW;
	 
		delay_cycles( 1 );
	
		mySPI_SendData(WRITE);

		mySPI_SendData(temp.adres.adres_h);

		mySPI_SendData(temp.adres.adres_l);
		
		while(cnt--)
		{
			mySPI_SendData(*writeData);
			writeData++;
			temp.adress++;
		}
	
		delay_cycles( 1 );
	
		MFRC522_CS_HIGH;
		delay_cycles( 1 );
	
		RF_Write_Disable();
	
		return temp.adress;

 }
 
 void RF_Write_Enable(void)
 {
 
	MFRC522_CS_LOW;
	
	delay_cycles( 1 ); 
	

	delay_cycles( 1 ); 
	
	MFRC522_CS_HIGH;
 }
 
 void RF_Write_Disable(void)
 {
	MFRC522_CS_LOW;
	
	delay_cycles( 1 ); 
	


	delay_cycles( 1 ); 
	
	MFRC522_CS_HIGH;
 }

 
 uint8_t M0_SPI_Send(uint8_t data) 
 {
	 uint8_t temp;
	//Fill ouOput buffer with data
	SPI_SendData8(SPI1, data);
	//Wait for transmission to complete
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
	//Wait for received data to complete
	while (!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	//Wait for SPI to be ready
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY));
	//Return data from buffer
	temp=SPI_ReceiveData8(SPI1);
	 
	 return temp;
}//
 


uint8_t read_status(void)
{
		uint8_t val;
	
		MFRC522_CS_LOW; //CS low
	
		delay_cycles( 1 );
		
		val=SPI_ReceiveData8(SPI1);
	
		delay_cycles( 1 );

	 
		MFRC522_CS_HIGH;//CS high
		return val;	
}

void write_status(void)
{
	
		MFRC522_CS_LOW; //CS low
	 
	
		MFRC522_CS_HIGH;//CS high
}

void delay_cycles( uint8_t a)
{
	uint16_t bb=1000;
	while(bb--);

	

}

uint8_t mySPI_GetData(void)
{
 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_SendData8(SPI1, 0x00); //Dummy byte to generate clock
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
 
	return  SPI_ReceiveData8(SPI1);
}

void mySPI_SendData( uint8_t data)
{
 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_SendData8(SPI1, data);
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	
	SPI_ReceiveData8(SPI1);
 
}

uint16_t	RF_blok_oku(uint16_t adress,uint8_t * data , uint8_t size)
{
	
	uint8_t sayac=0;
	
	data11 temp;
	
	temp.adress=adress;

	
	for(sayac=0;sayac<size;sayac++)
	{
	
		*data=RF_ReadRegister(&temp);
		data++;
		temp.adress++;
		
	}

	
	return (uint16_t) temp.adress;
}

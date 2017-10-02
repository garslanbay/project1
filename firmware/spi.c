#include "spi.h"

//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI1口初始化
void SPI1_Init(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  //使能GPIOA的时钟
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_0); //PA5复用为 SPI1_SCLK
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_0); //PA6复用为 SPI1_MISO
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_0); //PA7复用为 SPI1_MOSI
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_7;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//CE
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//SPI1_ReadWriteByte(0xff);//启动传输		 
}   
//SPI1速度设置函数
//SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
//fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	SPI1->CR1&=0XFFC7;//位3-5清零，用来设置波特率
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI1速度 
	SPI_Cmd(SPI1,ENABLE); //使能SPI1
} 
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		 			 
 uint8_t temp=0;
	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);//等待发送区空  
	
	SPI_SendData8(SPI1, TxData); //通过外设SPIx发送一个byte  数据
	
	
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{}
	
	temp=(uint8_t)SPI1->DR;
 
	return temp; //返回通过SPIx最近接收的数据	
 		    
}

uint8_t mySPI_GetData(void)
{
	uint8_t temp=0;

	
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_SendData8(SPI1, 0x00); //Dummy byte to generate clock
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY));
	SPI_ReceiveData8(SPI1);
	temp=(uint8_t) (SPI1->DR>>8);
	return  temp;
}








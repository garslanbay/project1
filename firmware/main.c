/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_gpio.h"
#include "24l01.h"


#define adc_bound		(uint32_t)0x40012440
#define vref					*((uint16_t*)0x1FFFF7BA)

uint16_t data5[150];
uint8_t adc_sayac=0;
uint16_t Adc_Data[3];
_Bool data_ok = 0,control=0 ;
uint8_t sayac=0;

void delay(uint16_t delayy);
void init_Adc(void);
void dma1(void);
void Perform_measurement(void);
void init_I2C1(void);
void GPIO_init1(void);
extern float get_temp(float pt_value);
extern void NRF24L01_Init(void);
extern uint8_t NRF24L01_Read_Reg(uint8_t reg);

static void EXTI_Config(void);


uint16_t sayac1=0;
uint8_t reg=0,reg2=0;

tip tip1[16];
tip tip2;
int main(void)
{
	int8_t bitval=0;
	  GPIO_InitTypeDef GPIO_InitStructure;
	
		
	  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOBEN, ENABLE);
		
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5  ;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
		GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	SysTick_Config((uint32_t)SysTick_CLKSource_HCLK_Div8/300);
	
	GPIO_init1();
	init_Adc();	
	init_I2C1();
	NRF24L01_Init();
	NRF24L01_RX_Mode();
	EXTI_Config();
	//Si7013_Detect((uint8_t*)&tip1);
		
	tip2.bits.bit7= tip1[0].bits.bit7;
	tip2.bits.bit6= tip1[2].bits.bit7;
	tip2.bits.bit5= tip1[4].bits.bit7;
	tip2.bits.bit4= tip1[6].bits.bit7;
	
	
	//NRF24L01_Write_Reg(WRITE_REG_NRF|CONFIG,0x03);
	reg2=NRF24L01_Read_Reg(CONFIG);
	

	
	
	//reg2=ARD(100,200);
	
  while (1)
  {
		
		Perform_measurement();
		__WFI();
		
		
			
			
  }  
}

void delay(uint16_t delayy)
{
	uint16_t temp=0;
	
	for(temp=0;temp<delayy;temp++)
	{
		__nop();
		__nop();
		__nop();
	}


}
uint8_t cal_fac=0;
void init_Adc(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		ADC_InitTypeDef adc;
		NVIC_InitTypeDef int_;
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
		
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    adc.ADC_Resolution=ADC_Resolution_12b;
    adc.ADC_DataAlign=ADC_DataAlign_Right;
    adc.ADC_ContinuousConvMode=ENABLE;
    adc.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_TRGO;
    adc.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_None;
    adc.ADC_ScanDirection=ADC_ScanDirection_Upward;


    ADC_Init(ADC1,&adc);
    ADC_ClockModeConfig(ADC1,ADC_ClockMode_SynClkDiv2);
    

    ADC_ChannelConfig(ADC1,ADC_Channel_0,ADC_SampleTime_239_5Cycles  );
		ADC_ChannelConfig(ADC1,ADC_Channel_1,ADC_SampleTime_239_5Cycles  );
		ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint , ADC_SampleTime_239_5Cycles);
		
		ADC_VrefintCmd(ENABLE);
		//ADC_TempSensorCmd(ENABLE);
		
    cal_fac=ADC_GetCalibrationFactor(ADC1);

    ADC_Cmd(ADC1,ENABLE);

    //ADC_DMACmd(ADC1,ENABLE);
		
		int_.NVIC_IRQChannel=ADC1_COMP_IRQn;
		int_.NVIC_IRQChannelCmd=ENABLE;
		int_.NVIC_IRQChannelPriority=4;
		
		NVIC_Init(&int_);
		
		ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
		ADC_ITConfig(ADC1,ADC_IT_EOSEQ,ENABLE);


    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

    //ADC_StartOfConversion(ADC1);

}

void dma1(void)
{
	DMA_InitTypeDef	dma;
	
	NVIC_InitTypeDef int_;
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		dma.DMA_BufferSize=3;
		//dma.DMA_DIR=DMA_DIR_PeripheralDST;
		dma.DMA_DIR=DMA_DIR_PeripheralSRC;
		dma.DMA_MemoryInc=DMA_MemoryInc_Enable;
		dma.DMA_Mode=DMA_Mode_Circular;
		//dma.DMA_PeripheralBaseAddr=tim3_bound ;
		dma.DMA_PeripheralBaseAddr=adc_bound;
		dma.DMA_MemoryBaseAddr =(uint32_t)&Adc_Data;
		dma.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;
		dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord;
		dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
		dma.DMA_Priority=DMA_Priority_High;
		dma.DMA_M2M=DISABLE;
	
		DMA_Init(DMA1_Channel1,&dma);
		DMA_Cmd(DMA1_Channel1, ENABLE); 
		
		DMA_ClearITPendingBit(DMA_IT_TC);
		
		DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
		
		int_.NVIC_IRQChannel=DMA1_Channel1_IRQn;
		int_.NVIC_IRQChannelCmd=ENABLE;
		int_.NVIC_IRQChannelPriority=3;
		NVIC_Init(&int_);
}//

float Bat_voltage=0.0f,Gas_voltage=0.0f;
float Vdda=3.35f;

void Perform_measurement(void)
{
    uint32_t temp1_bat=0;
		uint32_t temp2_gas=0;
		uint32_t Temp3_vdda=0;
		
	
		uint8_t sayac11=0;
	
		adc_sayac=0;
	
		ADC_StartOfConversion(ADC1);
	
		while(data_ok==0);
		data_ok=0;
	
		for(sayac=0;sayac<150;sayac+=3)
		{

		//VSense=vref;
			temp1_bat +=  data5[sayac];
		//VSense=vref;
			temp2_gas += data5[sayac+1];
		
			Temp3_vdda += data5[sayac+2];
		}
	
		
			
	
		temp1_bat= temp1_bat / 50;
		temp2_gas = temp2_gas /50;
		Temp3_vdda = Temp3_vdda/50;
		
		Vdda = 3.3f * ((float)vref / (float)Temp3_vdda);
		Bat_voltage= Vdda*2 *(temp1_bat/4096.0f);
		Gas_voltage=Vdda*2 * (temp2_gas/4096.0f);
		
		ADC_StopOfConversion(ADC1);
		
		if(Bat_voltage>3.5f)
		{
				GPIO_SetBits(GPIOB,GPIO_Pin_5);
		}
		else
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);
		}
		if(Bat_voltage>3.8f)
		{
				GPIO_SetBits(GPIOB,GPIO_Pin_4);
		}
		else
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_4);
		}
		if(Bat_voltage>4.1f)
		{
				//GPIO_SetBits(GPIOB,GPIO_Pin_3);
		}
		else
		{
			//GPIO_ResetBits(GPIOB,GPIO_Pin_3);
		}
}

void GPIO_init1(void)
{
	
	GPIO_InitTypeDef gpio_init;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	
	
	gpio_init.GPIO_Mode=GPIO_Mode_OUT;
	gpio_init.GPIO_OType=GPIO_OType_PP;
	gpio_init.GPIO_Pin=_5v_pin;
	gpio_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
	gpio_init.GPIO_Speed=GPIO_Speed_10MHz;
	
	GPIO_Init( _5v_port,&gpio_init);
	
	gpio_init.GPIO_Pin=_Gas_pin;
	
	GPIO_Init(_Gas_port,&gpio_init);
	
	GPIO_SetBits(_5v_port,_5v_pin);
	GPIO_SetBits(_Gas_port,_Gas_pin);
}

void init_I2C1(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOBEN,ENABLE);

	
	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7 
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1); // SDA
	
	// configure I2C1 
	I2C_InitStruct.I2C_Timing = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_InitStruct.I2C_AnalogFilter= I2C_AnalogFilter_Enable;
	I2C_InitStruct.I2C_DigitalFilter=0;
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

static void EXTI_Config(void)
{
	
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


  /* Configure PC13 pins as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Connect EXTI13 Line to PC13 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource10);

  /* Configure EXTI13 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI4_15 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
  

}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

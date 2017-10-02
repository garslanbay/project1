/**
  ******************************************************************************
  * @file    Template/stm32f0xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f0xx_it.h"
#include "main.h"

uint16_t a;
extern uint8_t tus,tus2,basildi;
uint16_t one_sec=0;


extern uint16_t data5[150];
extern uint8_t adc_sayac;
extern _Bool data_ok,control;
extern float Get_Int_Temp(void);

void get_rtcc(void);
/** @addtogroup STM32F0xx_DISCOVERY_Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	
		if(a>=1000)
	{
		a=0;
		one_sec=1;
	}
	a++;

	
	
}

void ADC1_IRQHandler(void)
{
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==SET)
	{
		ADC_ClearITPendingBit(ADC1,ADC_FLAG_EOC);
		
		data5[adc_sayac++]=ADC_GetConversionValue(ADC1);
		
		if(adc_sayac==150)
		{
				data_ok=1;
				adc_sayac=0;
		}
		//TIM3->CCR1=a>>1;
		//TIM3->CCR2=a>>1;
		//TIM3->CCR3=a>>1;
		//TIM3->CCR4=a>>1;
	
	}
	
		if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOSEQ)==SET)
	{
		ADC_ClearITPendingBit(ADC1,ADC_FLAG_EOSEQ);
		

	
	}
	
	


}//


void EXTI4_15_IRQHandler(void)
{
	int8_t bitval=0;
	bitval=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3);
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    /* Clear the EXTI line 13 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line10);
		GPIO_WriteBit(GPIOB,GPIO_Pin_3,!bitval);
		
		
  }
}






/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

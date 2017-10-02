/**
  ******************************************************************************
  * @file     Template/main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx.h"
#include "Si7021.h"
//#include "NRF24L01.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

typedef union _tip
{
	uint8_t byte;
	
	struct
	{
		unsigned bit0:1;
		unsigned bit1:1;
		unsigned bit2:1;
		unsigned bit3:1;
		unsigned bit4:1;
		unsigned bit5:1;
		unsigned bit6:1;
		unsigned bit7:1;
	}bits;


}tip;

#define _rcc_5v	RCC_AHBPeriph_GPIOA
#define _5v_pin	GPIO_Pin_2
#define _5v_port	GPIOA

#define _rcc_Gas	RCC_AHBPeriph_GPIOA
#define _Gas_pin	GPIO_Pin_11
#define _Gas_port	GPIOA

#define IRQ_RCC		RCC_AHBPeriph_GPIOA
#define IRQ_port	GPIOA
#define IRQ_pin		GPIO_Pin_10



#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

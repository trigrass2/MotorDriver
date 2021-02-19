/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_14
#define ERROR_LED_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOC
#define BRAKE_CTRL_Pin GPIO_PIN_3
#define BRAKE_CTRL_GPIO_Port GPIOA
#define VOLTAGE_CTRL_Pin GPIO_PIN_4
#define VOLTAGE_CTRL_GPIO_Port GPIOC
#define STATUS2_LED_Pin GPIO_PIN_5
#define STATUS2_LED_GPIO_Port GPIOC
#define HALL1_Pin GPIO_PIN_9
#define HALL1_GPIO_Port GPIOE
#define HALL2_Pin GPIO_PIN_11
#define HALL2_GPIO_Port GPIOE
#define HALL3_Pin GPIO_PIN_13
#define HALL3_GPIO_Port GPIOE
#define HALL1_GPIO_Pin GPIO_PIN_8
#define HALL1_GPIO_GPIO_Port GPIOD
#define HALL2_GPIO_Pin GPIO_PIN_9
#define HALL2_GPIO_GPIO_Port GPIOD
#define HALL3_GPIO_Pin GPIO_PIN_10
#define HALL3_GPIO_GPIO_Port GPIOD
#define ADC_TRIGGER_Pin GPIO_PIN_2
#define ADC_TRIGGER_GPIO_Port GPIOD
#define ADC_TRIGGER_EXTI_IRQn EXTI2_IRQn
#define NEGATIVE_SW_Pin GPIO_PIN_5
#define NEGATIVE_SW_GPIO_Port GPIOB
#define POSITIVE_SW_Pin GPIO_PIN_6
#define POSITIVE_SW_GPIO_Port GPIOB
#define HOME_SW_Pin GPIO_PIN_7
#define HOME_SW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define EXTI2_INT_TP_H					(GPIOB->BSRR |= GPIO_PIN_8) //	Set -> High
#define EXTI2_INT_TP_L					(GPIOB->BSRR |= GPIO_PIN_8<<16) //	Reset -> Low (TP1))
#define ADC_INT_TP_H						(GPIOB->BSRR |= GPIO_PIN_9) //	Set -> High
#define ADC_INT_TP_L						(GPIOB->BSRR |= GPIO_PIN_9<<16) //	Reset -> Low (TP1))
#define RUN_LED_TOGGLE					(GPIOC->ODR ^= GPIO_PIN_13) 	//BLUE:RUN_LED
#define STATUS2_LED_TOGGLE				(GPIOC->ODR ^= GPIO_PIN_5) 	//BLUE:RUN_LED
#define RED_LED_ON						(GPIOC->BSRR |= GPIO_PIN_14<<16)	//	Reset -> Low -> Red Led On
#define RED_LED_OFF						(GPIOC->BSRR |= GPIO_PIN_14)
#define GREEN_LED_ON					(GPIOC->BSRR |= GPIO_PIN_15<<16) //	Reset -> Low -> orange Led On
#define GREEN_LED_OFF					(GPIOC->BSRR |= GPIO_PIN_15)
#define BRAKE_OFF						(GPIOA->BSRR |= GPIO_PIN_3<<16) 	//	Reset -> Low -> Brake Off
#define BRAKE_ON						(GPIOA->BSRR |= GPIO_PIN_3)
#define VOLTAGE_CTRL_OFF				(GPIOC->BSRR |= GPIO_PIN_4<<16) 	//	Reset -> Low -> Voltage Off
#define VOLTAGE_CTRL_ON					(GPIOC->BSRR |= GPIO_PIN_4)

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

//#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

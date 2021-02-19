/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#if defined(HEXA_BD_10A_V10) //single leg hexar version
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_14
#define ERROR_LED_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOC
#define VOLTAGE_CTRL_Pin GPIO_PIN_3
#define VOLTAGE_CTRL_GPIO_Port GPIOA
#define NEGATIVE_LIMIT_Pin GPIO_PIN_4
#define NEGATIVE_LIMIT_GPIO_Port GPIOA
#define POSITIVE_LIMIT_Pin GPIO_PIN_5
#define POSITIVE_LIMIT_GPIO_Port GPIOA
#define EMERGENCY_Pin GPIO_PIN_6
#define EMERGENCY_GPIO_Port GPIOA
#define PWM_EN_PWM4_L_Pin GPIO_PIN_4
#define PWM_EN_PWM4_L_GPIO_Port GPIOC
#define BRAKE_Pin GPIO_PIN_5
#define BRAKE_GPIO_Port GPIOC
#define HOME_SW_Pin GPIO_PIN_5
#define HOME_SW_GPIO_Port GPIOB

#define EXTI2_INT_TP_H					(GPIOB->BSRR |= GPIO_PIN_8) //	Set -> High
#define EXTI2_INT_TP_L					(GPIOB->BSRR |= (GPIO_PIN_8 << 16)) //	Reset -> Low (TP1))
#define ADC_INT_TP_H						(GPIOB->BSRR |= GPIO_PIN_9) //	Set -> High
#define ADC_INT_TP_L						(GPIOB->BSRR |= (GPIO_PIN_9 << 16)) //	Reset -> Low (TP1))
#define RUN_LED_TOGGLE					(GPIOC->ODR ^= GPIO_PIN_13) 	//BLUE:RUN_LED
#define STATUS2_LED_TOGGLE				//(GPIOC->ODR ^= GPIO_PIN_5) 	//BLUE:RUN_LED

#define RED_LED_ON						(GPIOC->BSRR |= (GPIO_PIN_14 << 16))	//	Reset -> Low -> Red Led On
#define RED_LED_OFF						(GPIOC->BSRR |= GPIO_PIN_14)
#define GREEN_LED_ON					(GPIOC->BSRR |= (GPIO_PIN_15 << 16)) //	Reset -> Low -> Green Led On
#define GREEN_LED_OFF					(GPIOC->BSRR |= GPIO_PIN_15)
#define GREEN_LED_TOGGLE				(GPIOC->ODR ^= GPIO_PIN_15) 	//GREEN:STATUS1_LED
#define BRAKE_ON							(GPIOA->BSRR |= GPIO_PIN_3) 	//	Set -> High -> Brake On
#define BRAKE_OFF						(GPIOA->BSRR |= (GPIO_PIN_3 << 16))

#elif defined(HEXA_BD_10A_V20)
#define RUN_LED_Pin GPIO_PIN_13
#define RUN_LED_GPIO_Port GPIOC
#define ERROR_LED_Pin GPIO_PIN_14
#define ERROR_LED_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOC
#define STATUS2_LED_Pin GPIO_PIN_5
#define STATUS2_LED_GPIO_Port GPIOC
#define VOLTAGE_CTRL_Pin GPIO_PIN_4
#define VOLTAGE_CTRL_GPIO_Port GPIOC
#define NEGATIVE_LIMIT_Pin GPIO_PIN_5
#define NEGATIVE_LIMIT_GPIO_Port GPIOB
#define POSITIVE_LIMIT_Pin GPIO_PIN_6
#define POSITIVE_LIMIT_GPIO_Port GPIOB
#define EMERGENCY_Pin GPIO_PIN_14
#define EMERGENCY_GPIO_Port GPIOB
#define PWM_EN_PWM4_L_Pin GPIO_PIN_12
#define PWM_EN_PWM4_L_GPIO_Port GPIOC
#define BRAKE_Pin GPIO_PIN_3
#define BRAKE_GPIO_Port GPIOA
#define HOME_SW_Pin GPIO_PIN_7
#define HOME_SW_GPIO_Port GPIOB

#define EXTI2_INT_TP_H					(GPIOB->BSRR |= GPIO_PIN_8) //	Set -> High
#define EXTI2_INT_TP_L					(GPIOB->BSRR |= (GPIO_PIN_8 << 16)) //	Reset -> Low (TP1))
#define ADC_INT_TP_H						(GPIOB->BSRR |= GPIO_PIN_9) //	Set -> High
#define ADC_INT_TP_L						(GPIOB->BSRR |= (GPIO_PIN_9 << 16)) //	Reset -> Low (TP1))
#define RUN_LED_TOGGLE					(GPIOC->ODR ^= GPIO_PIN_13) 	//BLUE:RUN_LED
#define STATUS2_LED_TOGGLE			  	(GPIOC->ODR ^= GPIO_PIN_5) 	//ORANGE:RUN_LED
#define RED_LED_ON						(GPIOC->BSRR |= (GPIO_PIN_14 << 16))	//	Reset -> Low -> Red Led On
#define RED_LED_OFF						(GPIOC->BSRR |= GPIO_PIN_14)
#define GREEN_LED_ON					(GPIOC->BSRR |= (GPIO_PIN_15 << 16)) //	Reset -> Low -> Green Led On
#define GREEN_LED_OFF					(GPIOC->BSRR |= GPIO_PIN_15)
#define GREEN_LED_TOGGLE				(GPIOC->ODR ^= GPIO_PIN_15) 	//GREEN:STATUS1_LED
#define BRAKE_ON							(GPIOA->BSRR |= GPIO_PIN_3) 	//	Set -> High -> Brake On
#define BRAKE_OFF						(GPIOA->BSRR |= (GPIO_PIN_3 << 16))
#define VOLTAGE_CTRL_ON					(GPIOC->BSRR |= GPIO_PIN_4) 	//	Set -> High -> Brake On
#define VOLTAGE_CTRL_OFF				(GPIOC->BSRR |= (GPIO_PIN_4 << 16))

#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

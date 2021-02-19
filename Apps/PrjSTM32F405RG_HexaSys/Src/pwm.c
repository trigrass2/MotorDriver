#include "stm32f4xx_hal.h"
#include "Pwm.h"
#include "Peripheral.h"

extern TIM_HandleTypeDef htim8;
void EnablePwm(void)
{
	//	Set -> High
 #if defined(HEXA_BD_10A_V10) //single leg hexar version
	GPIOC->BSRR |= GPIO_PIN_4;
#else
#endif
	SetPwm(0, 0, 0);
}

void DisablePwm(void)
{
	//	Reset -> Low
#if defined(HEXA_BD_10A_V10) //single leg hexar version
	GPIOC->BSRR |= (GPIO_PIN_4 << 16);
#else
#endif

	SetPwm(-1, -1, -1);
}

void SetPwm(int16_t pwm1, int16_t pwm2, int16_t pwm3)
{
	if(pwm1 > MAX_PWM)		pwm1 = MAX_PWM;
	if(pwm2 > MAX_PWM)		pwm2 = MAX_PWM;
	if(pwm3 > MAX_PWM)		pwm3 = MAX_PWM;
	
  	if(pwm1 < 0) {
		//	PWM Disable -> Idle
		TIM8->CCER &= (uint16_t)(~((uint16_t)(TIM_CCER_CC1E|TIM_CCER_CC1NE)));
		TIM8->CCR1 = PWM_TIMER_PERIOD;
	}
	else if(pwm1 == 0) {
		TIM8->CCER |= (uint16_t)( ((uint16_t)(TIM_CCER_CC1E|TIM_CCER_CC1NE)));
		TIM8->CCR1 = PWM_TIMER_PERIOD;
	}
	else {
		//	PWM Enable -> Active
		TIM8->CCER |= (uint16_t)( ((uint16_t)(TIM_CCER_CC1E|TIM_CCER_CC1NE)));
		TIM8->CCR1 = PWM_TIMER_PERIOD - (pwm1 + DEAD_TIME_INV_2);
	}
	
	if(pwm2 < 0) {
		//	PWM Disable -> Idle
		TIM8->CCER &= (uint16_t)(~((uint16_t)(TIM_CCER_CC2E|TIM_CCER_CC2NE)));
		TIM8->CCR2 = PWM_TIMER_PERIOD;
	}
	else if(pwm2 == 0) {
		//	PWM Enable -> Active
		TIM8->CCER |= (uint16_t)( ((uint16_t)(TIM_CCER_CC2E|TIM_CCER_CC2NE)));
		TIM8->CCR2 = PWM_TIMER_PERIOD;
	}
	else {
		//	PWM Enable -> Active
		TIM8->CCER |= (uint16_t)( ((uint16_t)(TIM_CCER_CC2E|TIM_CCER_CC2NE)));
		TIM8->CCR2 =  PWM_TIMER_PERIOD - (pwm2 + DEAD_TIME_INV_2);
	}
	
	if(pwm3 < 0) {
		//	PWM Disable -> Idle
		TIM8->CCER &= (uint16_t)(~((uint16_t)(TIM_CCER_CC3E|TIM_CCER_CC3NE)));
		TIM8->CCR3 = PWM_TIMER_PERIOD;
	}
	else if(pwm3 == 0) {
		//	PWM Enable -> Active
		TIM8->CCER |= (uint16_t)( ((uint16_t)(TIM_CCER_CC3E|TIM_CCER_CC3NE)));
		TIM8->CCR3 = PWM_TIMER_PERIOD;
	}
	else {
		//	PWM Enable -> Active
		TIM8->CCER |= (uint16_t)( ((uint16_t)(TIM_CCER_CC3E|TIM_CCER_CC3NE)));
		TIM8->CCR3 = PWM_TIMER_PERIOD - (pwm3 + DEAD_TIME_INV_2);
	}
}

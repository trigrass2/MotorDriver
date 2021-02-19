//#include "gpio.h"
#include "Io.h"
#include "stm32h743xx.h"
#include "main.h"

uint32_t GetDigitalInput(void)
{
	uint32_t io = 0;
	
	//	Bit0 : Negative Limit
	//	Bit1 : Positive Limit
	//	Bit2 : Home
	//	Bit3 : Emergency
#if defined(HEXA_BD_10A_V10) //single leg hexar version
	io = (((GPIOA->IDR >> 4) & 0x3));			//	Negative/Positive Limit
	io |= (((GPIOB->IDR >> 5) & 0x1) << 2);		//	Home Sensor
	//io |= (((GPIOA->IDR >> 6) & 0x1) << 3);		//	Emergency
#elif defined(HEXA_BD_10A_V20)
	io = ((GPIOB->IDR >> 5) & 0x7);			//	Negative/Positive Limit/Home Sensor
	//io ^= 0x7;
#endif
	return io;
}

void SetDigitalOutput(uint32_t digitalOutput)
{

}

void TurnOnErrorLed(void)
{
	RED_LED_ON;//GPIOC->BSRR |= (GPIO_PIN_14 << 16);	//	Reset -> Low -> Red Led On
}

void TurnOffErrorLed(void)
{
	RED_LED_OFF;//GPIOC->BSRR |= GPIO_PIN_14;			//	Set -> High -> Red Led Off
}

void TurnOnStatusLed(uint8_t index)
{
	if(index == 0)	GREEN_LED_ON;//GPIOC->BSRR |= ((uint32_t)GPIO_PIN_15 << 16);	//	Reset -> Low -> Green Led On
}

void TurnOffStatusLed(uint8_t index)
{
	if(index == 0)	GREEN_LED_OFF;//GPIOC->BSRR |= GPIO_PIN_15;			//	Set -> High -> Green Led Off
}

void TurnOnStatusLedAll(void)
{
	GREEN_LED_ON;//GPIOC->BSRR |= ((uint32_t)GPIO_PIN_15 << 16);	//	Reset -> Low -> Led On
}

void TurnOffStatusLedAll(void)
{
	GREEN_LED_OFF;//GPIOC->BSRR |= GPIO_PIN_15;			//	Set -> High -> Led Off
}

void TurnOnRegenerativeBrake(void)
{
	#if defined(HEXA_BD_10A_V10)
	BRAKE_ON;//GPIOA->BSRR |= GPIO_PIN_3;			//	Set -> High -> Brake On
	#elif defined(HEXA_BD_10A_V20)
	VOLTAGE_CTRL_ON;
	#endif
}

void TurnOffRegenerativeBrake(void)
{
	#if defined(HEXA_BD_10A_V10)
	BRAKE_OFF;//GPIOA->BSRR |= (GPIO_PIN_3 << 16);	//	Reset -> Low -> Brake Off
	#elif defined(HEXA_BD_10A_V20)
	VOLTAGE_CTRL_OFF;
	#endif
}

void TurnOnBrake(void)
{
}

void ReadyBrake(void)
{
}

void TurnOffBrake(void)
{
}

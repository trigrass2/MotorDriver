#include "gpio.h"
#include "Io.h"

uint32_t GetDigitalInput(void)
{
	uint32_t io = 0;
	
	//	Bit0 : Negative Limit
	//	Bit1 : Positive Limit
	//	Bit2 : Home
	//	Bit3 : Emergency
	io = (((GPIOA->IDR >> 4) & 0x3));			//	Negative/Positive Limit
	io |= (((GPIOB->IDR >> 5) & 0x1) << 2);		//	Home Sensor
	//io |= (((GPIOA->IDR >> 6) & 0x1) << 3);		//	Emergency

	return io;
}

void SetDigitalOutput(uint32_t digitalOutput)
{

}

void TurnOnErrorLed(void)
{
	GPIOC->BSRR |= (GPIO_PIN_14 << 16);	//	Reset -> Low -> Led On
}

void TurnOffErrorLed(void)
{
	GPIOC->BSRR |= GPIO_PIN_14;			//	Set -> High -> Led Off
}

void TurnOnStatusLed(uint8_t index)
{
	if(index == 0)			GPIOC->BSRR |= ((uint32_t)GPIO_PIN_15 << 16);	//	Reset -> Low -> Led On
}

void TurnOffStatusLed(uint8_t index)
{
	if(index == 0)			GPIOC->BSRR |= GPIO_PIN_15;			//	Set -> High -> Led Off
}

void TurnOnStatusLedAll(void)
{
	GPIOC->BSRR |= ((uint32_t)GPIO_PIN_15 << 16);	//	Reset -> Low -> Led On
}

void TurnOffStatusLedAll(void)
{
	GPIOC->BSRR |= GPIO_PIN_15;			//	Set -> High -> Led Off
}

void TurnOnRegenerativeBrake(void)
{
	GPIOA->BSRR |= GPIO_PIN_3;			//	Set -> High -> Brake On
}

void TurnOffRegenerativeBrake(void)
{
	GPIOA->BSRR |= (GPIO_PIN_3 << 16);	//	Reset -> Low -> Brake Off
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
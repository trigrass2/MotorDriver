#pragma once

#include "math_def.h"


#define	NUMBER_OF_OVER_CURRENT		1000		//	50us*1000 -> 50ms

////////////////////////////////////////////////////////////////////////////////
//
//	EtherCAT KITECH
//
////////////////////////////////////////////////////////////////////////////////
#if defined(STM32F407IGH_ETHERCAT_KITECH)
#define	PWM_TIMER_PERIOD			4200	//	168MHz / 4200 / 2 = 20KHz -> 50us
#define	DEAD_TIME					168		//	1 / 168MHz * 2 * 84 = 1.0us
#define	DEAD_TIME_INV_2				(DEAD_TIME >> 1)
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.96f	//	96%

//	0.033V/A -> 0.033V : 1A = 3.3V : x -> x = 100A
//	3.3V : 100A = 3.3V/4096 : x -> 0.0244140625
#define	ADC_TO_CURRENT				0.0244140625f

//	Gain : 1.3 / (24.9 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.0162411949f

//	Gain : 1.3 / (33 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
//#define	ADC_TO_VOLTAGE				0.0212571364f

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.07326f

#define	MAX_CONT_CURRENT			25.0f
#define	MAX_OUTPUT_CURRENT			35.0f
#define	HW_CURRENT_LIMIT			40.0f

#define	MAX_DC_LINK_VOLTAGE			40.0f
#define	MIN_DC_LINK_VOLTAGE			10.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		35.0f

#define	CURRENT_CONTROLLER_PERIOD			0.00005f	//	50us
#define	CURRENT_CONTROLLER_FREQ				20000.0f	//	20KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	5

#define	VELOCITY_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	VELOCITY_CONTROLLER_FREQ			2000.0f		//	2KHz

#define	POSITION_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	POSITION_CONTROLLER_FREQ			2000.0f		//	2KHz

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 20KHz, Cut-off Frequency : 50KHz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI *200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 2KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 2KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

////////////////////////////////////////////////////////////////////////////////
//
//	Robotis
//
////////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F407GZ_ROBOTIS)
#define	PWM_TIMER_PERIOD			3360	//	168MHz / 3360 / 2 = 25KHz -> 40us
#define	DEAD_TIME					168		//	1 / 168MHz * 2 * 84 = 1.0us
#define	DEAD_TIME_INV_2				(DEAD_TIME >> 1)
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.95f	//	95%


//	0.11V/A -> 0.055V : 1A = 3.3V : x -> x = 60A
//	3.3V : 60A = 3.3V/4096 : x -> 0.0146484375
#define	ADC_TO_CURRENT				0.0146484375

//	Gain : 1.3 / (24.9 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
//#define	ADC_TO_VOLTAGE				0.0162411949f
#define	ADC_TO_VOLTAGE				0.0166411949f

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.07326f

#define	MAX_CONT_CURRENT			10.0f
#define	MAX_OUTPUT_CURRENT			20.0f
#define	HW_CURRENT_LIMIT			25.0f

#define	MAX_DC_LINK_VOLTAGE			55.0f
#define	MIN_DC_LINK_VOLTAGE			10.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		50.0f

#define	CURRENT_CONTROLLER_PERIOD			0.00004f	//	40us
#define	CURRENT_CONTROLLER_FREQ				25000.0f	//	25KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	12

#define	VELOCITY_CONTROLLER_PERIOD			0.00048f	//	0.48ms
#define	VELOCITY_CONTROLLER_FREQ			2083.3333f		//	1KHz

#define	POSITION_CONTROLLER_PERIOD			0.00048f	//	0.48ms
#define	POSITION_CONTROLLER_FREQ			2083.3333f		//	1KHz

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 50Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI *1000.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

///////////////////////////////////////////////////////////////////////////////
//
//		NSquare
//
///////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F407GZ_NSQUARE)
#define	PWM_TIMER_PERIOD			3360	//	168MHz / 6720 / 2 = 12.5KHz -> 80us
#define	DEAD_TIME					168		//	1 / 168MHz * 336 = 2.0us
#define	DEAD_TIME_MULTI_2			(DEAD_TIME << 1)
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME_MULTI_2)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.95f	//	95%


//	0.0264V/A -> 0.0264V : 1A = 3.3V : x -> x = 125A
//	3.3V : 125A = 3.3V/4096 : x -> 0.030525030525
#define	ADC_TO_CURRENT				0.030525030525f

//	Gain : 1.3 / (24.9 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.016237229767f

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.07326f

#define	MAX_CONT_CURRENT			25.0f
#define	MAX_OUTPUT_CURRENT			50.0f
#define	HW_CURRENT_LIMIT			50.0f

#define	MAX_DC_LINK_VOLTAGE			55.0f
#define	MIN_DC_LINK_VOLTAGE			10.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		50.0f

#define	CURRENT_CONTROLLER_PERIOD			0.00004f		//	40us
#define	CURRENT_CONTROLLER_FREQ				25000.0f		//	12.5KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	25

#define	VELOCITY_CONTROLLER_PERIOD			0.001f		//	1.0ms
#define	VELOCITY_CONTROLLER_FREQ			1000.0f		//	1.0KHz

#define	POSITION_CONTROLLER_PERIOD			0.001f		//	1.0ms
#define	POSITION_CONTROLLER_FREQ			1000.0f		//	1.0KHz

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 500Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 500.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)


////////////////////////////////////////////////////////////////////////////////
//
//	Eraetech
//
////////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F407GZ_ERAETECH)
#define	PWM_TIMER_PERIOD			16800	//	168MHz / 16800 / 2 = 5KHz -> 200us
#define	DEAD_TIME					336		//	1 / 168MHz * 336 = 4.0us
#define	DEAD_TIME_MULTI_2			672
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME_MULTI_2)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.98f	//	95%


//	0.0132V/A -> 0.0132V : 1A = 3.3V : x -> x = 250A
//	3.3V : 250A = 3.3V/4096 : x -> 0.06103515625
#define	ADC_TO_CURRENT				0.06103515625f

//	Gain : 8.45 / (990.0 + 8.45) = 0.008463117832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.095197074942

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.07326f

#define	MAX_CONT_CURRENT			30.0f
#define	MAX_OUTPUT_CURRENT			60.0f
#define	HW_CURRENT_LIMIT			70.0f

#define	MAX_DC_LINK_VOLTAGE			330.0f
#define	MIN_DC_LINK_VOLTAGE			10.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		350.0f

#define	CURRENT_CONTROLLER_PERIOD			0.0002f		//	200us
#define	CURRENT_CONTROLLER_FREQ				5000.0f		//	5KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	5

#define	VELOCITY_CONTROLLER_PERIOD			0.001f		//	1.0ms
#define	VELOCITY_CONTROLLER_FREQ			1000.0f		//	1.0KHz

#define	POSITION_CONTROLLER_PERIOD			0.001f		//	1.0ms
#define	POSITION_CONTROLLER_FREQ			1000.0f		//	1.0KHz

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 5KHz, Cut-off Frequency : 200Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 5KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 5KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

////////////////////////////////////////////////////////////////////////////////
//
//	Rohau
//
////////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F405RG_ROHAU)
#define	PWM_TIMER_PERIOD			4200	//	168MHz / 4200 / 2 = 20KHz -> 50us
#define	DEAD_TIME					84		//	1 / 168MHz * 2 * 84 = 1.0us
#define	DEAD_TIME_INV_2				(DEAD_TIME >> 1)
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.96f	//	96%


//	1-Axis
//	R = 0.007,	R1 = 33.0K, R2 = 3.9K, Gain = 33.0K / 3.9K = 8.46153846
//	1[V] = x[A] * (R * Gain), x[A] = 1[V] / (R * Gain) = 16.883116883[A]
//	3.3V = 16.883116883[A]*.3.3 = 50.64935
//	3.3V : 50.64935A = 3.3V/4096 : x -> 0.012368583797f
//#define	ADC_TO_CURRENT				0.012368583797f

//	2-Axes
//	R = 0.03,	R1 = 21.0K, R2 = 10.5K, Gain = 22K / 10.5K = 2.095238
//	1[V] = x[A] * (R * Gain), x[A] = 1[V] / (R * Gain) = 15.9090909[A]
//	3.3V = 15.9090909[A]*.3.3 = 52.5
//	3.3V : 52.5A = 3.3V/4096 : x -> 0.0128051282
#define	ADC_TO_CURRENT				0.0128051282f

//	Gain : 1.3 / (24.9 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.0162411949f


//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.07326f

#define	MAX_CONT_CURRENT			10.0f
#define	MAX_OUTPUT_CURRENT			20.0f
#define	HW_CURRENT_LIMIT			25.0f

#define	MAX_DC_LINK_VOLTAGE			50.0f
#define	MIN_DC_LINK_VOLTAGE			10.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		45.0f

#define	CURRENT_CONTROLLER_PERIOD			0.00005f	//	50us
#define	CURRENT_CONTROLLER_FREQ				20000.0f	//	20KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	10

#define	VELOCITY_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	VELOCITY_CONTROLLER_FREQ			2000.0f		//	2KHz

#define	POSITION_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	POSITION_CONTROLLER_FREQ			2000.0f		//	2KH

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI *200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

////////////////////////////////////////////////////////////////////////////////
//
//	HexaSys
//
////////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F405RG_HEXASYS)
#define	PWM_TIMER_PERIOD			4200	//	168MHz / 4200 / 2 = 20KHz -> 50us
//#define	PWM_TIMER_PERIOD			1680	//	168MHz / 1680 / 2 = 50KHz -> 20us
#define	DEAD_TIME					84		//	1 / 168MHz * 2 * 84 = 1.0us
#define	DEAD_TIME_INV_2				(DEAD_TIME >> 1)
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.98f	//	98%
//#define	PWM_EFFICIENCY				0.90f	//	90%

#if defined(HEXA_BD_10A_V10) //single leg hexar version
//	0.066V/A -> 0.066V : 1A = 3.3V : x -> x = 50A
//	3.3V : 50A = x/4095 : x -> 0.01221001221
#define	ADC_TO_CURRENT				0.01221001221f
//	3.3V : 20A = x/4095 : x -> 0.01221001221
//#define	ADC_TO_CURRENT				0.00488400488f //pjg--180717
#elif defined(HEXA_BD_10A_V20) //STM32F405RG_10A_ROHAU_CONTROLER
// gain : 20 V/V
// 3.3V:4096 = x : 1, => x = 3.3V/4095,
#define	ADC_TO_CURRENT				0.013427734f
#endif

//	Gain : 1.3 / (33 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.0212571364f

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.08056640625f

#define	MAX_CONT_CURRENT			10.0f
#define	MAX_OUTPUT_CURRENT			18.0f
#define	HW_CURRENT_LIMIT			20.0f

#define	MAX_DC_LINK_VOLTAGE			65.0f
#define	MIN_DC_LINK_VOLTAGE			12.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		60.0f

#define	CURRENT_CONTROLLER_PERIOD			0.00005f	//	50us
#define	CURRENT_CONTROLLER_FREQ				20000.0f	//	20KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	10

#define	VELOCITY_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	VELOCITY_CONTROLLER_FREQ			2000.0f		//	2KHz

#define	POSITION_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	POSITION_CONTROLLER_FREQ			2000.0f		//	2KH

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI *200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

////////////////////////////////////////////////////////////////////////////////
//
//	Step Drive
//
////////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F405RG_STEP_DRIVE)
#define	PWM_TIMER_PERIOD			3000	//	168MHz / 3000 / 2 = 28KHz -> 35.7142857us
#define	DEAD_TIME					84		//	1 / 168MHz * 84 * 2= 0.5us * 2 = 1us
#define	DEAD_TIME_INV_2				42		//	(DEAD_TIME >> 1)
#define	MAX_PWM						2874	//	(PWM_TIMER_PERIOD - DEAD_TIME - DEAD_TIME_INV_2)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.958f	//	95.8%

//	0.110V/A -> 0.110V : 1A = 3.3V : x -> x = 30A
//	3.3V : 30A = 3.3V/4095 : x -> 0.007326007326
#define	ADC_TO_CURRENT				0.007326007326f

//	Gain : 1.3 / (33 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.0212571364f

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.08056640625f

#define	MAX_CONT_CURRENT			5.0f
#define	MAX_OUTPUT_CURRENT			10.0f
#define	HW_CURRENT_LIMIT			12.0f

#define	MAX_DC_LINK_VOLTAGE			65.0f
#define	MIN_DC_LINK_VOLTAGE			10.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		39.0f

#define	CURRENT_CONTROLLER_PERIOD			0.0000357142857f	//	35.7142857us
#define	CURRENT_CONTROLLER_FREQ				28000.0f			//	28KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	14

#define	VELOCITY_CONTROLLER_PERIOD			0.0005f	//	0.5ms
#define	VELOCITY_CONTROLLER_FREQ			2000.0f		//	2KHz

#define	POSITION_CONTROLLER_PERIOD			0.0005f	//	0.5ms
#define	POSITION_CONTROLLER_FREQ			2000.0f		//	2KH

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI *100.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

////////////////////////////////////////////////////////////////////////////////
//
//	DST Circle 100W
//
////////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F407IGH_CIRCLE_100W)
#define	PWM_TIMER_PERIOD			4200	//	168MHz / 4200 / 2 = 20KHz -> 50us
#define	DEAD_TIME					84		//	1 / 168MHz * 2 * 84 = 1.0us
#define	DEAD_TIME_INV_2				(DEAD_TIME >> 1)
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.98f	//	98%
//#define	PWM_EFFICIENCY				0.90f	//	90%

//	0.110V/A -> 0.110V : 1A = 3.3V : x -> x = 30A
//	3.3V : 30A = 3.3V/4095 : x -> 0.007326007326
#define	ADC_TO_CURRENT				0.007326007326f

//	Gain : 1.3 / (33 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.0212571364f

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.08056640625f

#define	MAX_CONT_CURRENT			5.0f
#define	MAX_OUTPUT_CURRENT			10.0f
#define	HW_CURRENT_LIMIT			12.0f

#define	MAX_DC_LINK_VOLTAGE			65.0f
#define	MIN_DC_LINK_VOLTAGE			12.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		36.0f

#define	CURRENT_CONTROLLER_PERIOD			0.00005f	//	50us
#define	CURRENT_CONTROLLER_FREQ				20000.0f	//	20KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	10

#define	VELOCITY_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	VELOCITY_CONTROLLER_FREQ			2000.0f		//	2KHz

#define	POSITION_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	POSITION_CONTROLLER_FREQ			2000.0f		//	2KH

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI *200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)


////////////////////////////////////////////////////////////////////////////////
//
//	KITECH
//
////////////////////////////////////////////////////////////////////////////////
#elif defined(STM32F407ZE_ETHERCAT)
#define	PWM_TIMER_PERIOD			4200	//	168MHz / 4200 / 2 = 20KHz -> 50us
#define	DEAD_TIME					84		//	1 / 168MHz * 2 * 84 = 1.0us
#define	DEAD_TIME_INV_2				(DEAD_TIME >> 1)
#define	MAX_PWM						(PWM_TIMER_PERIOD - DEAD_TIME)
#define MAX_PWM_INV_2				(MAX_PWM >> 1)
#define	PWM_EFFICIENCY				0.98f	//	98%
//#define	PWM_EFFICIENCY				0.90f	//	90%

//	0.110V/A -> 0.110V : 1A = 3.3V : x -> x = 30A
//	3.3V : 30A = 3.3V/4095 : x -> 0.007326007326
#define	ADC_TO_CURRENT				0.007326007326f

//	Gain : 1.3 / (33 + 1.3) = 0.04961832
//	result * 3.3 / 4096 / Gain
#define	ADC_TO_VOLTAGE				0.03790087464f

//	ADC 변환:		1V --> 1365 
//	전압-온도 관계:	1V = 100'C
#define	ADC_TO_TEMPERATURE 			0.08056640625f

#define	MAX_CONT_CURRENT			5.0f
#define	MAX_OUTPUT_CURRENT			10.0f
#define	HW_CURRENT_LIMIT			12.0f

#define	MAX_DC_LINK_VOLTAGE			65.0f
#define	MIN_DC_LINK_VOLTAGE			10.0f
#define	CUT_OFF_DC_LINK_VOLTAGE		36.0f

#define	CURRENT_CONTROLLER_PERIOD			0.00005f	//	50us
#define	CURRENT_CONTROLLER_FREQ				20000.0f	//	20KHz
#define	CURRENT_MOVING_AVERAGE_FILTER_NUM	10

#define	VELOCITY_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	VELOCITY_CONTROLLER_FREQ			2000.0f		//	2KHz

#define	POSITION_CONTROLLER_PERIOD			0.0005f		//	0.5ms
#define	POSITION_CONTROLLER_FREQ			2000.0f		//	2KH

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	VOLTAGE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI *200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	TEMPERATURE_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 25KHz, Cut-off Frequency : 200Hz
#define	CURRENT_LOW_PASS_FILTER_COEF		(CURRENT_CONTROLLER_PERIOD * 2.0f * M_PI * 200.0f)

#endif

//	Low-Pass Filter Coefficient = Sampling Time * 2 * M_PI * Cut-off frequency
//	Sampling Time : 2KHz, Cut-off Frequency : 5Hz
#define	ELEC_THETA_LOW_PASS_FILTER_COEF		(VELOCITY_CONTROLLER_PERIOD * 2.0f * M_PI *5.0f)

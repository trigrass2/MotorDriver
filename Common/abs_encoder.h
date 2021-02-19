/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H
#define __ENCODER_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
typedef enum {
  ENC_SSI_POSITAL = 0,
  ENC_TAMAGAWA = 1
} ENC_TYPE;

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
#define SRAM_BANK1_NE1_ADDR                 ((uint32_t)0x60000000)
#define SRAM_BANK1_NE2_ADDR                 ((uint32_t)0x64000000)
#define SRAM_BANK1_NE3_ADDR                 ((uint32_t)0x68000000)
#define SRAM_BANK1_NE4_ADDR                 ((uint32_t)0x6C000000)

#define DEF_ENC_RQ_BUSY      HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)//(GPIOD->IDR & GPIO_PIN_8)   
#define DEF_ENC_RX_BUSY      HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)//(GPIOD->IDR & GPIO_PIN_9)   
#define DEF_ENC_TIMEOUT      HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)//(GPIOD->IDR & GPIO_PIN_10)   
#define DEF_ENC_INT          HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)//(GPIOE->IDR & GPIO_PIN_0)   

#define ENC_DATA_ID0_SINGLE		0x02
#define ENC_DATA_ID1_MULTI		0x8A
#define ENC_DATA_ID2_ENCID		0x92
#define ENC_DATA_ID3_ALL		0x1A
#define ENC_DATA_ID6_WRITE		0x32
#define ENC_DATA_IDD_READ		0xEA
#define ENC_DATA_ID7_RESETTING		0xBA
#define ENC_DATA_ID8_RESETTING		0xC2
#define ENC_DATA_IDC_RESETTING		0x62

#define ENC_CONTOL_FIELD		0x02

typedef struct _STEncStatusBitsDesc_t_		// bits  description
{     
	uint16_t	bInformation:4;			// bit 0-3	Encoder Information
	uint16_t	bEncError:2;			// bit 4-5	Encoder error
	uint16_t	bCommAlarm:2;			// bit 6-7	Communication alarm
	uint16_t	bReserved:8;			// bit 8-15	reserved
}STEncStatusBitsDesc;

typedef union _UEncStatusDesc_t_{
	STEncStatusBitsDesc	bits;
	unsigned char		byte;
}UEncStatusDesc;

typedef struct _STEncAlarmBitsDesc_t_		// bits  description
{     
	uint16_t	bOverSpeed:1;			      // bit 0	Overspeed
	uint16_t	bFullAbsoluteStatus:1;	// bit 1	Full absolute status
	uint16_t	bCountingError:1;		    // bit 2	Counting error
	uint16_t	bCounterOverflow:1;		  // bit 3	Counter overflow
	uint16_t	bReserved0:1;			      // bit 4	"0"
	uint16_t	bMultiturnError:1;		  // bit 5	Multiturn error
	uint16_t	bBatteryError:1;		    // bit 6	Battery error
	uint16_t	bBatteryAlarm:1;		    // bit 7	Battery alarm
	uint16_t	bReserved:8;			      // bit 8-15	reserved
}STEncAlarmBitsDesc;

typedef struct _ST_AbsEncAlarmBitsDesc_t_		// bits  description
{     
	uint16_t	bOverSpeed:1;			      // bit 0	Overspeed
	uint16_t	bBusyFlag:1;	          // bit 1	Buys flag
	uint16_t	bSTerror:1;		          // bit 2	Counting error
	uint16_t	bCounterOverflow:1;		  // bit 3	Counter overflow
	uint16_t	bReserved0:1;			      // bit 4	"0"
	uint16_t	bPSerror:1;		          // bit 5	PS (Multi/Single-turn) error
	uint16_t	bMTError:1;		          // bit 6	Multiturn error
	uint16_t	bBatteryAlarm:1;		    // bit 7	Battery alarm
	uint16_t	bReserved:8;			      // bit 8-15	reserved
}ST_AbsEncAlarmBitsDesc;

typedef union _UEncAlarmDesc_t_{
	ST_AbsEncAlarmBitsDesc	bits;
	unsigned char		byte;
}UEncAlarmDesc;


// USART3 ***** Hardware dependant code
#define TAMAGAWA_RX_ENABLE  (GPIOD->BSRR |= (uint32_t)GPIO_PIN_10 << 16U)
#define TAMAGAWA_TX_DISABLE (GPIOD->BSRR |= (uint32_t)GPIO_PIN_10 << 16U)

#define TAMAGAWA_RX_DISABLE (GPIOD->BSRR |= (uint32_t)GPIO_PIN_10)
#define TAMAGAWA_TX_ENABLE  (GPIOD->BSRR |= (uint32_t)GPIO_PIN_10)

// USART1 ***** Hardware dependant code
/*
#define TAMAGAWA_RX_ENABLE  (GPIOG->BSRR |= (uint32_t)GPIO_PIN_15 << 16U)
#define TAMAGAWA_TX_DISABLE (GPIOG->BSRR |= (uint32_t)GPIO_PIN_15 << 16U)

#define TAMAGAWA_RX_DISABLE (GPIOG->BSRR |= (uint32_t)GPIO_PIN_15)
#define TAMAGAWA_TX_ENABLE  (GPIOG->BSRR |= (uint32_t)GPIO_PIN_15)
*/

#define TAMAGAWA_STATUS_EA0_ERROR 0x20
#define TAMAGAWA_STATUS_EA1_ERROR 0x10


#define TAMAGAWA_ERROR_NONE   0x00
#define TAMAGAWA_ERROR_OVSPD  0x01
#define TAMAGAWA_ERROR_BUSY   0x02
#define TAMAGAWA_ERROR_STERR  0x04
#define TAMAGAWA_ERROR_OVF    0x08

#define TAMAGAWA_ERROR_PSERR  0x20
#define TAMAGAWA_ERROR_MTERR  0x40
#define TAMAGAWA_ERROR_BATT   0x80


typedef struct STEncDataDesc_t_		// bits  description
{     
	UEncStatusDesc	status;
	UEncAlarmDesc	  alarm;
	unsigned char 	encID;
	unsigned long	  singleTurn;
	long	  multiTurn;
  long	  pulse;
	unsigned char   bCF;
}STEncDataDesc;

extern UART_HandleTypeDef *_huart_abs_enc;
extern STEncDataDesc _abs_encoder;
extern uint8_t _abs_enc_dma_buf[11];
int8_t init_TAMAGAWA_abs_encoder(UART_HandleTypeDef *huart);
int8_t request_data_TAMAGAWA_abs_encoder( );
int8_t reset_TAMAGAWA_abs_encoder( );
uint8_t get_data_TAMAGAWA_abs_encoder(unsigned long *single, long *multi, long *pulse);


uint8_t init_Encoder(ENC_TYPE type);
uint8_t reset_Encoder(void);
uint8_t set_EncoderSSI(uint8_t clock, uint8_t resolution);

extern int16_t _encMultiTurnValue;
extern int16_t _encSingleTurnValue;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ENCODER_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

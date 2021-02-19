#ifndef __can_comm_H
#define __can_comm_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define MAX_CAN_QUEUE_SIZE	64

typedef struct CanMsg_t {
    uint32_t 	id;
    uint32_t    len;
    uint8_t     data[8];
} CanMsg_t;
	 
typedef struct CanQueue_t
{
    CanMsg_t data[MAX_CAN_QUEUE_SIZE];
    int start;
    int end;
    int length;
    int lock;
}CanQueue_t;

extern CanQueue_t CanTxMsgQueue;
extern CanQueue_t CanRxMsgQueue;

 
int8_t coe_flushMbox(void);
void Can_config(CAN_HandleTypeDef* hcan, uint32_t id);
void Can_receive_start(CAN_HandleTypeDef* hcan);

int8_t Can_get_length_Rx_queue(void);
void Can_init_Rx_queue(void);
void Can_clear_Rx_queue(void);
int8_t Can_msg_insert_Rx_queue(uint32_t id, uint8_t *data, uint32_t len);
int8_t Can_insert_Rx_queue(CanMsg_t *pMsg);
int8_t Can_extract_Rx_queue(CanMsg_t *pMsg);
int8_t Can_get_length_Tx_queue(void);
void Can_init_Tx_queue(void);
void Can_clear_Tx_queue(void);
int8_t Can_insert_Tx_queue(CanMsg_t *pMsg);
int8_t Can_extract_Tx_queue(CanMsg_t *pMsg);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__can_comm_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

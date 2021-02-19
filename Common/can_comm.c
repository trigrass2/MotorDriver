/* Includes ------------------------------------------------------------------*/
#include "can_comm.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "ServoControllerInterface.h"
CanQueue_t CanTxMsgQueue;       ///< M4 CAN tx message queue
CanQueue_t CanRxMsgQueue;       ///< M4 CAN rx message queue

static CanTxMsgTypeDef        CanTxMessage;     ///< M4 CAN tx message
static CanRxMsgTypeDef        CanRxMessage;     ///< M4 CAN rx message

//CAN_HandleTypeDef *coe_hcan;            ///< M4 CAN handle pointer for CANopen

/// @brief M4 CAN Configuration function
/// @param type : CAN type, e.g. CAN1, CAN2
/// @param id : CAN ID
/// @return None
void Can_config(CAN_HandleTypeDef *hcan, uint32_t id)
{
  CAN_FilterConfTypeDef  sFilterConfig;
  uint8_t filterNum = 0;
  if(hcan->Instance == CAN2)
    filterNum = 14;

  hcan->pTxMsg = &CanTxMessage;
  hcan->pRxMsg = &CanRxMessage;    

  // Configure the CAN Filter
  sFilterConfig.FilterNumber = filterNum;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000 + id;       // NODE_ID_MASK
  sFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = filterNum;
  
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    while(1)
    {
    }
  }
  
 /* 
  sFilterConfig.FilterNumber = 15;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//CAN_FILTERMODE_IDLIST;//CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000 + id;       // NODE_ID_MASK
  sFilterConfig.FilterFIFOAssignment = CAN_FIFO1;
  sFilterConfig.FilterActivation = ENABLE;//ENABLE;//DISABLE;
  sFilterConfig.BankNumber = 14;
  
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    while(1)
    {
    }
  }  
 */
  // Configure Transmission process
  hcan->pTxMsg->StdId = 0x321;
  hcan->pTxMsg->ExtId = 0x012345;
  hcan->pTxMsg->RTR = CAN_RTR_DATA;
  hcan->pTxMsg->IDE = CAN_ID_STD;
  //hcan1.pTxMsg->IDE = CAN_ID_EXT;  
  hcan->pTxMsg->DLC = 8;
  
  //coe_hcan = hcan;
}
/*
int8_t coe_flushMbox(void)
{
  CanMsg_t msg;
  
  if(Can_get_length_Rx_queue() > 0)
  {
    Can_extract_Rx_queue(&msg);
    ProcessSdoCan(&msg.id, msg.data, &msg.len);
	coe_hcan->pTxMsg->StdId = msg.id;
	memcpy(&coe_hcan->pTxMsg->Data, msg.data, msg.len);
    HAL_CAN_Transmit(coe_hcan, 10);
  }  
  return 1;
}
*/
/// @brief M4 CAN receive start function
/// @param type : CAN type, e.g. CAN1, CAN2
/// @return None
void Can_receive_start(CAN_HandleTypeDef *hcan)
{
  if(HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
  {
  }  
  //if(HAL_CAN_Receive_IT(hcan, CAN_FIFO1) != HAL_OK)
  //{
  //}    
}

/// @brief M4 CAN get received message length
/// @param None
/// @return int8_t : CAN received message length
int8_t Can_get_length_Rx_queue(void)
{
  return CanRxMsgQueue.length;
}

/// @brief M4 CAN get received message queue initialization function
/// @param None
/// @return None
void Can_init_Rx_queue(void)
{
  CanQueue_t *pQueue;
  pQueue = &CanRxMsgQueue;
  pQueue->start = 0;
  pQueue->end = 0;
  pQueue->length = 0;
  pQueue->lock = 0;
  memset(pQueue, 0, sizeof(CanQueue_t));
}

/// @brief M4 CAN received message queue clear function
/// @param None
/// @return None
void Can_clear_Rx_queue(void)
{
  Can_init_Rx_queue( );
}

/// @brief M4 CAN received message queue insert function
/// @param id : CAN ID
/// @param data : CAN received data
/// @param len : CAN received data length
/// @return int8_t : Inserted data length at rx queue
int8_t Can_msg_insert_Rx_queue(uint32_t id, uint8_t *data, uint32_t len)
{
  CanMsg_t msg;
  uint8_t i;
  
  msg.id = id;
  msg.len = len;
  for(i=0; i<len; i++)
    msg.data[i] = (uint8_t)data[i];
  
  return Can_insert_Rx_queue(&msg);
}

/// @brief M4 CAN received message queue insert function
/// @param pMsg : CAN message struct data
/// @return int8_t : Inserted data length at rx queue
int8_t Can_insert_Rx_queue(CanMsg_t *pMsg)
{
  CanQueue_t *pQueue;
  pQueue = &CanRxMsgQueue;
  if(pQueue->length < MAX_CAN_QUEUE_SIZE)
  {
    memcpy(&pQueue->data[pQueue->end++], pMsg, sizeof(CanMsg_t));
    pQueue->length++;
    if(pQueue->end >= MAX_CAN_QUEUE_SIZE) 
      pQueue->end = 0;
    return 1;
  }else
    return -1;
}

/// @brief M4 CAN received message queue extract function
/// @param pMsg : CAN message struct data
/// @return int8_t : Extract data length at rx queue
int8_t Can_extract_Rx_queue(CanMsg_t *pMsg)
{
  CanQueue_t *pQueue;
  pQueue = &CanRxMsgQueue;
  if(pQueue->length > 0)
  {
    memcpy(pMsg, &pQueue->data[pQueue->start++], sizeof(CanMsg_t));
    pQueue->length--;
    if(pQueue->start >= MAX_CAN_QUEUE_SIZE) 
      pQueue->start = 0;
    return 1;
  }
  else	
    return -1;
}

/// @brief M4 CAN get transmitted message length
/// @param None
/// @return int8_t : CAN transmitted message length
int8_t Can_get_length_Tx_queue(void)
{
  return CanTxMsgQueue.length;
}

/// @brief M4 CAN get transmit message queue initialization function
/// @param None
/// @return None
void Can_init_Tx_queue(void)
{
  CanQueue_t *pQueue;
  pQueue = &CanTxMsgQueue;
  pQueue->start = 0;
  pQueue->end = 0;
  pQueue->length = 0;
  pQueue->lock = 0;
  memset(pQueue, 0, sizeof(CanQueue_t));
}

/// @brief M4 CAN transmit message queue clear function
/// @param None
/// @return None
void Can_clear_Tx_queue(void)
{
  Can_init_Tx_queue( );
}

/// @brief M4 CAN transmit message queue insert function
/// @param pMsg : CAN message struct data
/// @return int8_t : Inserted data length at tx queue
int8_t Can_insert_Tx_queue(CanMsg_t *pMsg)
{
  CanQueue_t *pQueue;
  pQueue = &CanTxMsgQueue;
  if(pQueue->length < MAX_CAN_QUEUE_SIZE)
  {
    memcpy(&pQueue->data[pQueue->end++], pMsg, sizeof(CanMsg_t));
    pQueue->length++;
    if(pQueue->end >= MAX_CAN_QUEUE_SIZE) 
      pQueue->end = 0;
    return 1;
  }else
    return -1;
}

/// @brief M4 CAN received message queue extract function
/// @param pMsg : CAN message struct data
/// @return int8_t : Extract data length at tx queue
int8_t Can_extract_Tx_queue(CanMsg_t *pMsg)
{
  CanQueue_t *pQueue;
  pQueue = &CanTxMsgQueue;
  if(pQueue->length > 0)
  {
    memcpy(pMsg, &pQueue->data[pQueue->start++], sizeof(CanMsg_t));
    pQueue->length--;
    if(pQueue->start >= MAX_CAN_QUEUE_SIZE) 
      pQueue->start = 0;
    return 1;
  }
  else	
    return -1;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

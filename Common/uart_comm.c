#include "uart_comm.h"

#include "stm32f4xx_hal.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CanDef.h"
#include "queue.h"
#include "utils.h"
#include "ServoControllerInterface.h"

/// @def SERIAL_BUS_STX
/// @brief Serial start data
#define SERIAL_BUS_STX		0x02            
/// @def SERIAL_BUS_ETX
/// @brief Serial end data
#define SERIAL_BUS_ETX		0x03
/// @def SERIAL_BUS_HEADER
/// @brief Serial header data
#define SERIAL_BUS_HEADER	0xF0

/// @def SERIAL_PID_CAN_MSG
/// @brief Serial data format : CAN message (CANopen msg.)
#define SERIAL_PID_CAN_MSG                      0x00
/// @def SERIAL_PID_FIRMWARE_VER
/// @brief Serial data format : Firmware version data
#define SERIAL_PID_FIRMWARE_VER		        0x10
/// @def SERIAL_PID_LOG_MSG
/// @brief Serial data format : Leg message
#define SERIAL_PID_LOG_MSG		        0x20
/// @def SERIAL_PID_ACK
/// @brief Serial data format : Comm. Ack. data
#define SERIAL_PID_ACK                          0xFE
/// @def SERIAL_PID_ERROR_INVALID_VALUE
/// @brief Serial data format : Error data data
#define SERIAL_PID_ERROR_INVALID_VALUE 	        0xFF

unsigned int m_bRcvPacketDataLen = 0;           ///< Serial recieved packet data length
unsigned int m_bRcvDataLen = 0;                 ///< Serial recieved data length

UART_HandleTypeDef *handleUart;

Queue_t m_usrTxQueue;
Queue_t m_usrRxQueue;

uint8_t m_usrTxBuffer[2048];
uint8_t m_usrRxBuffer[64];

char uart_makeCheckSum(char pid, char *pData, unsigned int len);
void uart_transmitData(char pid, char *pData, unsigned int len);

void uart_tansmitDataHandlerUsingUart(void *port);
void uart_tansmitDataHandlerUsingUsbCDC( );

uint32_t nTransmitData = 0;

void uart_log_message(char *sz, ...)
{
	int nLength;
	static char arBuf[256] = {0, };  
	va_list arg_list;
  
	va_start(arg_list, sz);
	nLength = vsprintf(arBuf, sz, arg_list);

	va_end(arg_list);
	uart_transmitData(SERIAL_PID_LOG_MSG, arBuf, nLength);
}

void uart_initialize(void *huart)
{
	handleUart = (UART_HandleTypeDef *)huart;
	queue_initialize(&m_usrTxQueue);
	queue_initialize(&m_usrRxQueue);
}


void uart_resetError(void *huart)
{
  UART_HandleTypeDef *handle = (UART_HandleTypeDef *)huart;  
  if(handleUart == handle) 
  {
      __HAL_UART_FLUSH_DRREGISTER(handle);
      uart_receiveProcessStart(handle);
  }
}

void uart_tansmitDataHandlerUsingUart(void *huart)
{
	int cnt=0;
    UART_HandleTypeDef *handle = (UART_HandleTypeDef *)huart;  
    
	if(handleUart == NULL) {
		return;
	}
	
	if(handleUart == handle) 
    {
		if(HAL_UART_GetState(handle) != HAL_UART_STATE_BUSY_TX) {
			if(m_usrTxQueue.length > 0) {
				nTransmitData++;
				GPIOD->BSRR |= GPIO_PIN_2;	//	SET
				cnt = queue_extract_string(&m_usrTxQueue, (char *)m_usrTxBuffer, m_usrTxQueue.length);
				HAL_UART_Transmit_IT(handle, (uint8_t*)m_usrTxBuffer, cnt);     
			}
		}
	}
}

void uart_tansmitDataHandler(void *huart)
{
  UART_HandleTypeDef *handle = (UART_HandleTypeDef *)huart;  
  if(handleUart == handle) 
  {
	uart_tansmitDataHandlerUsingUart(handle);
  }
}

void uart_receiveProcessStart(void *huart)
{
  UART_HandleTypeDef *handle = (UART_HandleTypeDef *)huart;  
  if(handleUart == handle) 
  {
	HAL_UART_Receive_IT(handle, (uint8_t *)m_usrRxBuffer, 1);
  }  
}

void uart_insertQueueReceivedData(void *huart)
{
  UART_HandleTypeDef *handle = (UART_HandleTypeDef *)huart;  
  if(handleUart == handle) 
  {
    __HAL_UART_FLUSH_DRREGISTER(handle); // Clear the buffer to prevent overrun  
  
	uart_receiveProcessStart(handle);
	queue_insert_byte(&m_usrRxQueue, m_usrRxBuffer[0]);
  }
}

char uart_makeCheckSum(char pid, char *pData, unsigned int len)
{
	char bCrc = 0;
	unsigned int i;
	bCrc = pid;
	bCrc += len;
	for(i=0; i<len; i++) {
		bCrc += *(pData+i);
	}

	return bCrc;
}

void uart_transmitData(char pid, char *pData, unsigned int len)
{
	char bData[256];
	bData[0] = SERIAL_BUS_STX;
	bData[1] = SERIAL_BUS_HEADER;
	bData[2] = pid;
	bData[3] = len;
	memcpy(&bData[4], pData, len);
	bData[len+4] = uart_makeCheckSum(pid, pData, len);
	bData[len+5] = SERIAL_BUS_ETX;
	queue_insert_string(&m_usrTxQueue, bData, len+6);
}

void uart_tansmitCanMsg(CAN_MSG  *msg)
{
	char bData[30];
	
	memcpy(&bData[0], (char *)&msg->id, 4);
	bData[4] = msg->len;
	memcpy(&bData[5], msg->data, msg->len);
  
	uart_transmitData(SERIAL_PID_ACK, bData, msg->len+5);
}

void uart_canOpenMsgHandler(uint32_t id, uint8_t *data, uint8_t len)
{
	CAN_OPEN_SDO_MSG canOpenMsg;
	CAN_MSG  msg;
	memset(&msg, 0, sizeof(CAN_MSG ));
	
	if(len < 4) return;
	
	canOpenMsg.cmd = *(data);
	canOpenMsg.index = BytesToUint16((data+1));
	canOpenMsg.subIndex = *(data+3);
	memcpy(canOpenMsg.data, (data+4), len - 4);
	
	msg.len = ProcessSdo(&canOpenMsg);
	
	msg.id = (id & 0x7F) | 0x580;
	msg.data[0] = canOpenMsg.cmd;
	msg.data[1] = (uint8_t)(canOpenMsg.index & 0xFF);
	msg.data[2] = (uint8_t)((canOpenMsg.index >> 8) & 0xFF);
	msg.data[3] = canOpenMsg.subIndex;
	memcpy(&msg.data[4], &canOpenMsg.data[0], 4);
	msg.len += 4;
  
	uart_tansmitCanMsg(&msg);
}

void uart_pidDataHandler(char pid, char len, char *pData)
{
	CAN_MSG  msg;

	switch(pid) {
		case SERIAL_PID_CAN_MSG:
			//dbgPrint(">> Recieve Serial Data PID = SERIAL_PID_CAN_MSG");      
			memcpy(&msg.id, pData, 4);
			msg.len = pData[4];
			memcpy(&msg.data, &pData[5], msg.len);
			uart_canOpenMsgHandler(msg.id, msg.data, msg.len); 
			break;
      
		case SERIAL_PID_FIRMWARE_VER:
			//dbgPrint(">> Recieve Serial Data PID = SERIAL_PID_FIRMWARE_VER");
			break;

		case SERIAL_PID_LOG_MSG:
			//dbgPrint(">> Recieve Serial Data PID = SERIAL_PID_LOG_MSG");
			break;
      
		default:
			//ehs_dbgPrint("ERR: Invalid pid");
			break;
	}
}
 
void *uart_getReceiveQueuePtr()
{
	return (void *)(&m_usrRxQueue);
} 

void *uart_getTransmitQueuePtr()
{
	return (void *)(&m_usrTxQueue);
} 

int8_t uart_receivedDataHandler(void *pRxQueue)
{
	char bRcvData[64];
	char bRcvByte;
	Queue_t *pRcvQueue = (Queue_t *)pRxQueue;
	//Queue_t *pSendQueue = &m_usrTxQueue;

	if(pRcvQueue->length < 4) {
		return -1;
	}

	if(pRcvQueue->length >= 4) {
		queue_peek_string(pRcvQueue, bRcvData, pRcvQueue->start, 4);

		if(bRcvData[0] == SERIAL_BUS_STX) {
			if(bRcvData[1] == SERIAL_BUS_HEADER) {
				m_bRcvDataLen = bRcvData[3];
				m_bRcvPacketDataLen = m_bRcvDataLen + 6;
				if(pRcvQueue->length < m_bRcvPacketDataLen) {
					return -1;
				}
				else {
					if(queue_extract_string(pRcvQueue, bRcvData, m_bRcvPacketDataLen)) {
						if(uart_makeCheckSum(bRcvData[2], &bRcvData[4], m_bRcvDataLen) ==  bRcvData[m_bRcvPacketDataLen-2]) {
							uart_pidDataHandler(bRcvData[2], bRcvData[3], &bRcvData[4]);
							return 1;
						}
						else {
							//ehs_dbgPrint("ER : Comm. Message CRC Error!");
						}
						m_bRcvPacketDataLen = 0;
					}
				}
			}
			else {
				queue_extract_byte(pRcvQueue, &bRcvByte);
				m_bRcvPacketDataLen = 0;
				//ehs_dbgPrint("ER : RX Data 0x%x", bRcvByte);
				return -1;
			}
		}
		else {
			queue_extract_byte(pRcvQueue, &bRcvByte);
			//ehs_dbgPrint("ER : RX Data 0x%x", bRcvByte);
			m_bRcvPacketDataLen = 0;
			return -1;
		}
	}
	
    return -1;
}

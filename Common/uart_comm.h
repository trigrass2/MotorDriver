#pragma once

#include <stdint.h>

void uart_initialize(void *huart);
void uart_receiveProcessStart(void *huart);
void uart_resetError(void *huart);

void uart_tansmitDataHandler(void *huart);
int8_t uart_receivedDataHandler(void *pRxQueue);
void uart_insertQueueReceivedData(void *huart);
void uart_log_message(char *sz, ...);
void *uart_getReceiveQueuePtr();
void *uart_getTransmitQueuePtr();
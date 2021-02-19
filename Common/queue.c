/**
  ******************************************************************************
  * @file    queue.c
  * @author  KITECH WON Daehee
  * @date    7-7-2014
  * @brief   Queue    
  ******************************************************************************  
  */ 

#include "Queue.h"
#include <string.h>

void queue_initialize(Queue_t *pQueue)
{
  pQueue->start = 0;
  pQueue->end = 0;
  pQueue->length = 0;
  pQueue->lock = 0;
  memset(pQueue, 0, sizeof(Queue_t));

}

void queue_clear(Queue_t *pQueue)
{
  queue_initialize(pQueue);
}

int queue_get_space_free(Queue_t *pQueue)
{
  int size = MAX_QUEUE_SIZE - pQueue->length;
  return ( size < 0 ) ? 0 : size;
}

int queue_get_space_used(Queue_t *pQueue)
{
  return pQueue->length;                      
}

/// <summary>
/// description
/// </summary>
/// <param name="pQueue">description</param>
/// <param name="data">description</param>
/// <param name="len">description</param>
/// <returns>unsigned int : description</returns>
unsigned int queue_insert_string(Queue_t *pQueue, char *data, unsigned int len)
{
  unsigned int write_length = 0, i;

  if((MAX_QUEUE_SIZE - pQueue->length) >= len)
  {
    write_length = len;
  }
  else
    write_length = MAX_QUEUE_SIZE-pQueue->length;
  
  for(i=0 ; i<write_length ; i++)
  {
    pQueue->data[pQueue->end++] = *data++;
    pQueue->length++;
    if(pQueue->end >= MAX_QUEUE_SIZE)
      pQueue->end = 0;
  }
  return write_length;
}

unsigned int queue_insert_byte(Queue_t *pQueue, char data)
{
  if(pQueue->length < MAX_QUEUE_SIZE)
  {
    pQueue->data[pQueue->end++] = data;
    pQueue->length++;
    if(pQueue->end >= MAX_QUEUE_SIZE) 
            pQueue->end = 0;
    return 1;
  }else
    return 0;
}


unsigned int queue_extract_byte_isr(Queue_t *pQueue, char *data)
{
  if(pQueue->length > 0)
  {
    *data = pQueue->data[pQueue->start++];
    pQueue->length--;
    if(pQueue->start >= MAX_QUEUE_SIZE) 
            pQueue->start = 0;
    return 1;
  }
  else	
    return 0;
}

unsigned int queue_extract_byte(Queue_t *pQueue, char *data)
{
  if(pQueue->length > 0)
  {
    *data = pQueue->data[pQueue->start++];
    pQueue->length--;
    if(pQueue->start >= MAX_QUEUE_SIZE) 
            pQueue->start = 0;
    return 1;
  }
  else	
    return 0;
}

unsigned int queue_extract_string(Queue_t *pQueue, char *data, unsigned int len)
{
  unsigned int read_length = 0, i;

  if(pQueue->length > 0) 
  {
    if(pQueue->length >= len)
      read_length = len;
    else
      read_length = pQueue->length;

    for(i=0 ; i<read_length ; i++)
    {
      *data++ = pQueue->data[pQueue->start++];
      pQueue->length--;
      if(pQueue->start >= MAX_QUEUE_SIZE) 
        pQueue->start = 0;
    }
  }		
  return read_length;
}

unsigned int queue_peek_end_byte(Queue_t *pQueue,  char *data)
{
  unsigned int peekIndex;

  if(pQueue->length == 0)
    return 0;

  if(pQueue->end == 0) 
    peekIndex = MAX_QUEUE_SIZE-1;
  else
    peekIndex = pQueue->end-1;

  *data = pQueue->data[peekIndex];
  return 1;
}


unsigned int queue_peek_byte(Queue_t *pQueue,  char *data, unsigned int peekIndex)
{
  if(pQueue->length == 0)
    return 0;

  if(peekIndex >= MAX_QUEUE_SIZE) 
    return 0;

  if(pQueue->start < pQueue->end)
  {
    if((pQueue->start <= peekIndex) && (peekIndex < pQueue->end))
      *data = pQueue->data[peekIndex];
    else
      return 0;
  }else if(pQueue->start > pQueue->end)
  {
    if((peekIndex < pQueue->end) || (pQueue->start <= peekIndex))
      *data = pQueue->data[peekIndex];
    else
      return 0;
  }
  return 1;
}


unsigned int queue_peek_string(Queue_t *pQueue, char *data, unsigned int startIndex, unsigned int len)
{
  unsigned int i, iIndex, read_length=0;

  if(pQueue->length == 0)
    return 0;

  if(pQueue->length >= len)
    read_length = len;
  else
    read_length = pQueue->length;

  if(startIndex >= MAX_QUEUE_SIZE) 
    return 0;

  iIndex = startIndex;

  if(pQueue->start < pQueue->end)
  {
    if((pQueue->start <= startIndex) && (startIndex < pQueue->end))
    {
      for(i=0 ; i<read_length ; i++)
        *data++ = pQueue->data[iIndex++];
    }
    else
      return 0;
  }else if(pQueue->start > pQueue->end)
  {
    if((startIndex < pQueue->end) || (pQueue->start <= startIndex))
    {
      for(i=0 ; i<read_length ; i++)
      {
        *data++ = pQueue->data[iIndex++];
        if(iIndex >= MAX_QUEUE_SIZE) 
          iIndex = 0;
      }
    }
    else
      return 0;
  }
  
  return read_length;
}



unsigned int queue_delete_string(Queue_t *pQueue, unsigned int len)
{
  unsigned int iDeleted=0;

  if(pQueue->length < len)
    iDeleted = pQueue->length;
  else
    iDeleted = len;

  pQueue->length -= iDeleted;
  pQueue->start += iDeleted;
          
  if(pQueue->start >= MAX_QUEUE_SIZE) 
    pQueue->start -= MAX_QUEUE_SIZE;
  return iDeleted;
}

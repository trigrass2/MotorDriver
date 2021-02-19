#ifndef _QUEUE_H_
#define _QUEUE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_QUEUE_SIZE	128


typedef struct
{
	char data[MAX_QUEUE_SIZE];
	unsigned int start;
	unsigned int end;
	unsigned int length;
	unsigned char lock;
}Queue_t;

void queue_initialize(Queue_t *pQueue);
void queue_clear(Queue_t *pQueue);
int queue_get_space_free(Queue_t *pQueue);
int queue_get_space_used(Queue_t *pQueue);
unsigned int queue_insert_string(Queue_t *pQueue, char *data, unsigned int len);
unsigned int queue_insert_byte(Queue_t *pQueue, char data);
unsigned int queue_extract_byte(Queue_t *pQueue, char *data);
unsigned int queue_extract_string(Queue_t *pQueue, char *data, unsigned int len);
unsigned int queue_peek_byte(Queue_t *pQueue,  char *data, unsigned int peekIndex);
unsigned int queue_peek_end_byte(Queue_t *pQueue,  char *data);
unsigned int queue_peek_string(Queue_t *pQueue, char *data, unsigned int startIndex, unsigned int len);
unsigned int queue_delete_string(Queue_t *pQueue, unsigned int len);

unsigned int queue_extract_byte_isr(Queue_t *pQueue, char *data);
#ifdef __cplusplus
}
#endif

#endif	//_QUEUE_H_


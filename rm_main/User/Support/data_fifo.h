#ifndef __DATA_FIFO_H
#define __DATA_FIFO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "cmsis_os.h"

#define MUTEX_WAIT() \
do {\
	osMutexWait(pfifo->mutex, osWaitForever);\
} while(0)\

#define MUTEX_RELEASE() \
do {\
	osMutexRelease(pfifo->mutex);\
} while(0)\

typedef struct
{
	uint8_t		*start_addr;
	uint8_t		*end_addr;
	uint32_t	free;
	uint32_t	buf_size;	
	uint32_t	used;
	uint8_t		read_index;
	uint8_t		write_index;
	osMutexId	mutex;
} fifo_t;

fifo_t* FIFO_Create(uint32_t unit_cnt, osMutexId mutex);
void FIFO_Destory(fifo_t* pfifo);
int32_t FIFO_Init(fifo_t* pfifo, void* base_addr, uint32_t unit_cnt, osMutexId mutex);
int32_t FIFO_Put(fifo_t* pfifo, uint8_t element);
int32_t FIFO_Puts(fifo_t *pfifo, uint8_t *psource, uint32_t number);
uint8_t FIFO_Get(fifo_t* pfifo);
uint16_t FIFO_Gets(fifo_t* pfifo, uint8_t* source, uint8_t len);
uint8_t FIFO_Pre_Read(fifo_t* pfifo, uint8_t offset);
uint8_t FIFO_Is_Empty(fifo_t* pfifo);
uint8_t FIFO_Is_Full(fifo_t* pfifo);
uint32_t FIFO_Used_Count(fifo_t* pfifo);
uint32_t FIFO_Free_Count(fifo_t* pfifo);
uint8_t FIFO_Flush(fifo_t* pfifo);

#ifdef __cplusplus
}
#endif

#endif

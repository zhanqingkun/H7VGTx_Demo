#ifndef __DATA_FIFO_H
#define __DATA_FIFO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"
#include "stdint.h"

typedef struct
{
    void*     pbuff;       //首地址
    uint32_t  free;        //未使用单元数量
    uint32_t  buf_size;	   //FIFO单元大小
    uint32_t  buf_num;     //FIFO单元数量
    uint32_t  used;        //已使用单元数量
    uint32_t  read_index;  //读取单元索引
    uint32_t  write_index; //写入单元索引
    osMutexId mutex;       //对应的互斥锁
} fifo_t;

fifo_t*  FIFO_Create(uint32_t unit_num, uint32_t unit_size);
void     FIFO_Delete(fifo_t* pfifo);
int32_t  FIFO_Push(fifo_t* pfifo, const void* pdata);
int32_t  FIFO_Pushs(fifo_t *pfifo, const void* pdata, uint32_t number);
int32_t  FIFO_Pop(fifo_t* pfifo, void* pdata);
int32_t  FIFO_Pops(fifo_t* pfifo, void* pdata, uint32_t number);
int32_t  FIFO_PreRead(fifo_t* pfifo, void* pdata, uint32_t offset);
uint8_t  FIFO_IsEmpty(fifo_t* pfifo);
uint8_t  FIFO_IsFull(fifo_t* pfifo);
uint32_t FIFO_UsedCount(fifo_t* pfifo);
uint32_t FIFO_FreeCount(fifo_t* pfifo);
void     FIFO_Flush(fifo_t* pfifo);

#ifdef __cplusplus
}
#endif

#endif

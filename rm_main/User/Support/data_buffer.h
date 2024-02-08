#ifndef __DATA_BUFFER_H
#define __DATA_BUFFER_H

#include "stm32h7xx.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"

typedef struct
{
	void* pbuff;//缓存区的首地址
	uint32_t num;//num个size字节的数据单元
	uint32_t size;
	uint32_t write;//待写入单元索引，(write-1)指向最新数据
	uint8_t isfull;//缓存器满标志
} buffer_t;

buffer_t* Buffer_Create(uint32_t num, uint32_t size);
void Buffer_Delete(buffer_t** pbf);
void Buffer_Push(buffer_t* pbf, const void* pdata);
void* Buffer_Pop(buffer_t* pbf, uint32_t k);
uint32_t Buffer_Pops(buffer_t* pbf, void* pdata, uint32_t k, uint32_t num);
uint32_t Buffer_Used_Count(buffer_t* pbf);
void Buffer_Flush(buffer_t* pbf);

//两个滤滤器只适用于float数据单元的buffer
float Buffer_AVG_Filter(buffer_t* pbf, void* pdata, uint32_t num);
float Buffer_Lin_Filter(buffer_t* pbf, void* pdata, float* pweigth, uint32_t num);

#endif

#ifndef __DATA_BUFFER_H
#define __DATA_BUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

typedef struct
{
    void*    pbuff;       //缓存区的首地址
    uint32_t buf_num;     //缓存单元数量
    uint32_t buf_size;    //缓存单元长度
    uint32_t write_index; //待写入单元索引，(write-1)指向最新数据
    uint8_t  isfull;      //缓存器满标志
} buffer_t;

buffer_t* Buffer_Create(uint32_t num, uint32_t size);
void      Buffer_Delete(buffer_t* pbf);
int32_t   Buffer_Push(buffer_t* pbf, const void* pdata);
int32_t   Buffer_Get(buffer_t* pbf, void* pdata, uint32_t offset);
int32_t   Buffer_Gets(buffer_t* pbf, void* pdata, uint32_t offset, uint32_t number);
uint32_t  Buffer_UsedCount(buffer_t* pbf);
void      Buffer_Flush(buffer_t* pbf);

//这两个滤滤器只适用于float缓存单元长度的buffer
float Buffer_AVG_Filter(buffer_t* pbf, const void* pdata, uint32_t num);
float Buffer_Lin_Filter(buffer_t* pbf, const void* pdata, float* pweight, uint32_t num);

#ifdef __cplusplus
}
#endif

#endif

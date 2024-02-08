#include "data_fifo.h"
#include "stdlib.h"
#include "stdio.h"

#define ASSERT(x) do {while(!(x));} while(0)

fifo_t* FIFO_Create(uint32_t unit_cnt, osMutexId mutex)
{
	fifo_t *pfifo = NULL;
	uint8_t *base_addr = NULL;
	//检查输入参数
	ASSERT(0 != unit_cnt);
	pfifo = (fifo_t*) malloc(sizeof(fifo_t));
	if(NULL == pfifo)
		return (NULL);
	base_addr = malloc(unit_cnt);
	if(NULL == base_addr)
		return (NULL);
	FIFO_Init(pfifo, base_addr, unit_cnt, mutex);
	return (pfifo);
}

void FIFO_Destory(fifo_t* pfifo)
{
	//检查输入参数
	ASSERT(NULL != pfifo);
	ASSERT(NULL != pfifo->start_addr);
	//释放内存
	free(pfifo->start_addr);
	//删除互斥量
	osMutexDelete(pfifo->mutex);
	//释放fifo控制块内存
	free(pfifo);
	return;
}

int32_t FIFO_Init(fifo_t* pfifo, void* base_addr, uint32_t unit_cnt, osMutexId mutex)
{
	//检查输入参数
	ASSERT(NULL != pfifo);
	ASSERT(NULL != base_addr);
	ASSERT(0    != unit_cnt);
	
	pfifo->mutex = mutex;
	if (mutex != NULL)
	{
		//初始化fifo控制块
		pfifo->start_addr  = (uint8_t*) base_addr;
		pfifo->end_addr    = (uint8_t*) base_addr + unit_cnt - 1;
		pfifo->buf_size    = unit_cnt;
		pfifo->free        = unit_cnt;
		pfifo->used        = 0;
		pfifo->read_index  = 0;
		pfifo->write_index = 0;
		return 0;
	}
	else
		return -1;
}

//存入一个数据
//返回成功与否
int32_t FIFO_Put(fifo_t* pfifo, uint8_t element)
{
	//检查输入参数
	ASSERT(NULL != pfifo);
	if(0 >= pfifo->free)
	{
		return -1;//fifo已满，错误
	}
	MUTEX_WAIT();
	pfifo->start_addr[pfifo->write_index++] = element;
	pfifo->write_index %= pfifo->buf_size;
	pfifo->free--;
	pfifo->used++;
	MUTEX_RELEASE();
	return 0;
}

//存入多个数据
//返回成功存入的数据数量
int32_t FIFO_Puts(fifo_t *pfifo, uint8_t *psource, uint32_t number)
{
	int puts_num = 0;
	//检查输入参数
	ASSERT(NULL != pfifo);
	if(psource == NULL)
		return -1;
	MUTEX_WAIT();
	for(uint32_t i = 0; (i < number) && (pfifo->free > 0); i++)
	{
		pfifo->start_addr[pfifo->write_index++] = psource[i];
		pfifo->write_index %= pfifo->buf_size;
		pfifo->free--;
		pfifo->used++;
		puts_num++;
	}
	MUTEX_RELEASE();
	return puts_num;
}

//取出一个数据
//返回成功与否
uint8_t FIFO_Get(fifo_t* pfifo)
{
	uint8_t retval = 0;
	//检查输入参数
	ASSERT(NULL != pfifo);
	MUTEX_WAIT();
	retval = pfifo->start_addr[pfifo->read_index++];
	pfifo->read_index %= pfifo->buf_size;
	pfifo->free++;
	pfifo->used--;
	MUTEX_RELEASE();
	return retval;
}

//取出多个数据
//返回成功取出的数据数量
uint16_t FIFO_Gets(fifo_t* pfifo, uint8_t* source, uint8_t len)
{
	uint8_t retval = 0;
	//检查输入参数
	ASSERT(NULL != pfifo);
	MUTEX_WAIT();
	for(int i = 0; (i < len) && (pfifo->used > 0); i++)
	{
		source[i] = pfifo->start_addr[pfifo->read_index++];
		pfifo->read_index %= pfifo->buf_size;
		pfifo->free++;
		pfifo->used--;
		retval++;
	}
	MUTEX_RELEASE();
	return retval;
}

//获取fifo中任意一个数据且不删除
//返回数据
uint8_t FIFO_Pre_Read(fifo_t* pfifo, uint8_t offset)
{
    uint32_t index;
    //检查输入参数
    ASSERT(NULL != pfifo);
    if(offset > pfifo->used)
        return 0x00;
    else
    {
        index = ((pfifo->read_index + offset) % pfifo->buf_size);
        // Move Read Pointer to right position
        return pfifo->start_addr[index];
    }
}

//返回fifo是否空闲
uint8_t FIFO_Is_Empty(fifo_t* pfifo)
{
    //检查输入参数
    ASSERT(NULL != pfifo);
    return (0 == pfifo->used);
}

//返回fifo是否充满
uint8_t FIFO_Is_Full(fifo_t* pfifo)
{
    //检查输入参数
    ASSERT(NULL != pfifo);
    return (0 == pfifo->free);
}

//返回fifo已使用空间大小
uint32_t FIFO_Used_Count(fifo_t* pfifo)
{
    //检查输入参数
    ASSERT(NULL != pfifo);
    return (pfifo->used);
}

//返回fifo未使用空间大小
uint32_t FIFO_Free_Count(fifo_t* pfifo)
{
    //检查输入参数
    ASSERT(NULL != pfifo);
    return (pfifo->free);
}

//清空fifo
uint8_t FIFO_Flush(fifo_t* pfifo)
{
	//检查输入参数
	ASSERT(NULL != pfifo);
	MUTEX_WAIT();
	pfifo->free        = pfifo->buf_size;
	pfifo->used        = 0;
	pfifo->read_index  = 0;
	pfifo->write_index = 0;
	MUTEX_RELEASE();
	return 0;
}

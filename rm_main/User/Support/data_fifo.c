#include "data_fifo.h"
#include "stm32h7xx.h"
#include "string.h"
#include "stdlib.h"

//@breif  初始化FIFO结构体
//@param  pfifo    : FIFO结构体指针
//@param  base_addr: FIFO缓冲区首地址
//@param  unit_num : FIFO缓冲区单元数量
//@param  unit_size: FIFO缓冲区单元大小
//@retval 成功返回0，失败返回-1
static int32_t FIFO_Init(fifo_t* pfifo, void* base_addr, uint32_t unit_num, uint32_t unit_size)
{
    //检查输入参数
    assert_param(pfifo && base_addr && unit_num && unit_size);
    
    osMutexDef_t mute_def = {0};
    pfifo->mutex = osMutexCreate(&mute_def);
    if(pfifo->mutex != NULL)
    {
        pfifo->pbuff       = base_addr;
        pfifo->buf_num     = unit_num;
        pfifo->buf_size    = unit_size;
        pfifo->free        = unit_num;
        pfifo->used        = 0;
        pfifo->read_index  = 0;
        pfifo->write_index = 0;
        return 0;
    }
    else
        return -1;
}

//@breif  创建FIFO结构体
//@param  unit_num : FIFO缓冲区单元数量
//@param  unit_size: FIFO缓冲区单元大小
//@retval 成功返回FIFO指针，失败返回NULL
fifo_t* FIFO_Create(uint32_t unit_num, uint32_t unit_size)
{
    //检查输入参数
    assert_param(unit_num && unit_size);

    fifo_t* pfifo = NULL;
    uint8_t* base_addr = NULL;
    pfifo = (fifo_t*)malloc(sizeof(fifo_t));
    if(pfifo == NULL)
        return NULL;
    base_addr = malloc(unit_num * unit_size);
    if(base_addr == NULL)
    {
        free(pfifo);
        return NULL;
    }
    FIFO_Init(pfifo, base_addr, unit_num, unit_size);
    return pfifo;
}

//@breif  删除FIFO结构体
//@param  pfifo: FIFO结构体指针
//@retval None
void FIFO_Delete(fifo_t* pfifo)
{
    //检查输入参数
    assert_param(pfifo && pfifo->pbuff);
    //释放内存
    free(pfifo->pbuff);
    //删除互斥量
    osMutexDelete(pfifo->mutex);
    //释放fifo结构体内存
    free(pfifo);
}

//@breif  压入一个数据
//@param  pfifo: FIFO结构体指针
//@param  pdata: 压入数据指针
//@retval 成功返回0，失败返回-1
int32_t FIFO_Push(fifo_t* pfifo, const void* pdata)
{
    //检查输入参数
    assert_param(pfifo && pdata);
    if(pfifo->free <= 0)
        return -1;//fifo已满，错误
    osMutexWait(pfifo->mutex, osWaitForever);
    memcpy(&((uint8_t*)pfifo->pbuff)[pfifo->write_index * pfifo->buf_size], pdata, pfifo->buf_size);
    pfifo->write_index++;
    pfifo->write_index %= pfifo->buf_num;
    pfifo->free--;
    pfifo->used++;
    osMutexRelease(pfifo->mutex);
    return 0;
}

//@breif  压入多个数据
//@param  pfifo : FIFO结构体指针
//@param  pdata : 压入数据指针
//@param  number: 压入数据数量
//@retval 返回成功存入的数据数量，失败返回-1
int32_t FIFO_Pushs(fifo_t *pfifo, const void* pdata, uint32_t number)
{
    int32_t puts_num = 0;
    //检查输入参数
    assert_param(pfifo && pdata && number);
    if(pfifo->free <= 0)
        return -1;//fifo已满，错误
    for(uint32_t i = 0; (i < number) && (pfifo->free > 0); i++)
    {
        FIFO_Push(pfifo, &((uint8_t*)pdata)[i * pfifo->buf_size]);
        puts_num++;
    }
    return puts_num;
}

//@breif  取出一个数据
//@param  pfifo: FIFO结构体指针
//@param  pdata: 取出数据存放指针
//@retval 成功返回0，失败返回-1
int32_t  FIFO_Pop(fifo_t* pfifo, void* pdata)
{
    //检查输入参数
    assert_param(pfifo);
    if(pfifo->used <= 0)
        return -1;//FIFO已空，错误
    osMutexWait(pfifo->mutex, osWaitForever);
    memcpy(pdata, &((uint8_t*)pfifo->pbuff)[pfifo->read_index * pfifo->buf_size], pfifo->buf_size);
    pfifo->read_index++;
    pfifo->read_index %= pfifo->buf_num;
    pfifo->free++;
    pfifo->used--;
    osMutexRelease(pfifo->mutex);
    return 0;
}

//@breif  取出多个数据
//@param  pfifo : FIFO结构体指针
//@param  pdata : 取出数据存放指针
//@param  number: 取出数据数量
//@retval 返回成功取出数据的数量，失败返回-1
int32_t FIFO_Pops(fifo_t* pfifo, void* pdata, uint32_t number)
{
    int32_t pops_num = 0;
    //检查输入参数
    assert_param(pfifo && pdata && number);
    if(pfifo->used <= 0)
        return -1;//FIFO已空，错误
    for(int i = 0; (i < number) && (pfifo->used > 0); i++)
    {
        FIFO_Pop(pfifo, &((uint8_t*)pdata)[i * pfifo->buf_size]);
        pops_num++;
    }
    return pops_num;
}

//@breif  获取倒数第offset次压入的数据
//@param  pfifo : FIFO结构体指针
//@param  pdata : 取出数据存放指针
//@param  offset: 倒数第offset次，范围1 ~ pfifo->buf_num||pfifo->used
//@retval 成功返回0，失败返回-1
int32_t FIFO_PreRead(fifo_t* pfifo, void* pdata, uint32_t offset)
{
    
    //检查输入参数
    assert_param(pfifo && pdata);
    if(offset <= pfifo->write_index)//向前读取
        memcpy(pdata, &((uint8_t*)pfifo->pbuff)[(pfifo->write_index - offset) * pfifo->buf_size], pfifo->buf_size);
    else if(offset <= pfifo->buf_num && offset <= pfifo->used)//折回尾部读取
        memcpy(pdata, &((uint8_t*)pfifo->pbuff)[(pfifo->buf_num + pfifo->write_index - offset) * pfifo->buf_size], pfifo->buf_size);
    else//超出缓存容量
        return -1;
    return 0;
}

//@breif  FIFO是否空闲
//@param  pfifo : FIFO结构体指针
//@retval 空闲返回1，否则返回0
uint8_t FIFO_IsEmpty(fifo_t* pfifo)
{
    //检查输入参数
    assert_param(pfifo);
    return (pfifo->used == 0);
}

//@breif  FIFO是否充满
//@param  pfifo : FIFO结构体指针
//@retval 充满返回1，否则返回0
uint8_t FIFO_IsFull(fifo_t* pfifo)
{
    //检查输入参数
    assert_param(pfifo);
    return (pfifo->free == 0);
}

//@breif  FIFO已使用空间大小
//@param  pfifo : FIFO结构体指针
//@retval 返回fifo已使用空间大小
uint32_t FIFO_UsedCount(fifo_t* pfifo)
{
    //检查输入参数
    assert_param(pfifo);
    return pfifo->used;
}

//@breif  FIFO未使用空间大小
//@param  pfifo : FIFO结构体指针
//@retval 返回fifo未使用空间大小
uint32_t FIFO_FreeCount(fifo_t* pfifo)
{
    //检查输入参数
    assert_param(pfifo);
    return pfifo->free;
}

//@breif  清空FIFO所有空间
//@param  pfifo : FIFO结构体指针
//@retval None
void FIFO_Flush(fifo_t* pfifo)
{
    //检查输入参数
    assert_param(pfifo);
    osMutexWait(pfifo->mutex, osWaitForever);
    memset(pfifo->pbuff, 0, pfifo->buf_num * pfifo->buf_size);
    pfifo->free        = pfifo->buf_num;
    pfifo->used        = 0;
    pfifo->read_index  = 0;
    pfifo->write_index = 0;
    osMutexRelease(pfifo->mutex);
}

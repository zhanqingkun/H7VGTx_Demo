#include "data_buffer.h"
#include "stm32h7xx.h"
#include "string.h"
#include "stdlib.h"

//@breif  输入缓存单元个数和长度，创建缓存器
//@param  num : 创建缓存器所需缓存单元数量
//@param  size: 创建缓存器缓存单元长度
//@retval 成功返回缓存器指针，失败返回NULL
buffer_t* Buffer_Create(uint32_t num, uint32_t size)
{
    buffer_t* pbf = (buffer_t*)malloc(sizeof(buffer_t));
    void* pbuff = NULL;
    memset(pbf, 0, sizeof(buffer_t));
    if(pbf)
    {
        pbf->buf_num = num;
        pbf->buf_size = size;
        pbuff = calloc(num, size);//缓存器缓存实体，同时清零
        assert_param(pbuff);
        pbf->pbuff = pbuff;
        return pbf;
    }
    return NULL;
}

//@breif  删除缓存器，释放缓存器内存
//@param  pbf: 缓存器指针
//@retval None
void Buffer_Delete(buffer_t* pbf)
{
    free(pbf->pbuff);
    free(pbf);
    pbf = NULL;//防止指针被进一步使用
}

//@breif  将数据压入缓存器
//@param  pbf  : 缓存器指针
//@param  pdata: 压入数据指针
//@retval 成功返回0，失败返回-1
int32_t Buffer_Push(buffer_t* pbf, const void* pdata)
{
    assert_param(pbf);
    memcpy(&((uint8_t*)pbf->pbuff)[pbf->write_index * pbf->buf_size], pdata, pbf->buf_size);
    if(++pbf->write_index >= pbf->buf_num)
    {
        pbf->write_index = 0;
        pbf->isfull = 1;
    }
    return 0;
}

//@breif  获取倒数第offset次压入的数据
//@param  pbf   : 缓存器指针
//@param  pdata : 指向取出数据的指针
//@param  offset: 目标数据位置，范围1~num
//@retval 成功返回0，失败返回-1
int32_t Buffer_Get(buffer_t* pbf, void* pdata, uint32_t offset)
{
    assert_param(pbf && pdata);
    if(offset <= pbf->write_index)//向前读取
        memcpy(pdata, &((uint8_t*)pbf->pbuff)[(pbf->write_index - offset) * pbf->buf_size], pbf->buf_size);
    else if(offset <= pbf->buf_num && pbf->isfull == 1)//折回尾部读取
        memcpy(pdata, &((uint8_t*)pbf->pbuff)[(pbf->buf_num + pbf->write_index - offset) * pbf->buf_size], pbf->buf_size);
    else//超出缓存容量
        return -1;
    return 0;
}

//@breif  获取缓存区若干数据，数据从新到旧排列
//@param  pbf   : 缓存器指针
//@param  pdata : 指向取出数据的指针
//@param  offset: 目标数据位置，范围1~num
//@param  num   : 取出数据数量
//@retval 成功返回实际取出数量
//@note   传进函数里的数组指针一定要提前声明好！
int32_t Buffer_Gets(buffer_t* pbf, void* pdata, uint32_t offset, uint32_t number)
{
    assert_param(pbf);
    int32_t flag;
    uint32_t real_res_num = 0;
    for(uint32_t cnt = 0; cnt < number; ++cnt)
    {
        flag = Buffer_Get(pbf, &((uint8_t*)pdata)[cnt * pbf->buf_size], cnt + 1);
        if(flag == -1)//无法取出则跳出
            break;
        else 
            ++real_res_num;
    }
    return real_res_num;
}

//@breif  获取缓存器中当前数据数量
//@param  pbf: 缓存器指针
//@retval 返回当前数据数量
uint32_t Buffer_UsedCount(buffer_t* pbf)
{
	assert_param(pbf);
    uint32_t res;
    if(pbf->isfull)//缓存器已满
        res = pbf->buf_num;
    else
        res = pbf->write_index;
    return res;
}

//@breif  清空缓存区
//@param  pbf: 缓存器指针
//@retval None
void Buffer_Flush(buffer_t* pbf)
{
    assert_param(pbf);
    memset(pbf->pbuff, 0, pbf->buf_num * pbf->buf_size);
    pbf->write_index = 0;
    pbf->isfull = 0;
}

//@breif  实时更新平均滤波(压入数据同时滤波)
//@param  pbf  : 缓存器指针
//@param  pdata: 压入数据的指针
//@param  num  : 滤波数据长度
//@retval 返回平均滤波值
//@note   仅适用于float数据单元的buffer
float Buffer_AVG_Filter(buffer_t* pbf, const void* pdata, uint32_t num)
{
    assert_param(pbf);
    Buffer_Push(pbf, pdata);//存入最新的数据
    if(Buffer_UsedCount(pbf) <= num)//缓存区有效数据不足
        num = Buffer_UsedCount(pbf);//限制平均分母
    float temp_res = 0.0f;
    float* temp_array = NULL;
    temp_array = (float*)calloc(num, sizeof(float));//声明临时空间 同时清零
    if(temp_array)
    {
        Buffer_Gets(pbf, temp_array, 1, num);
        for(uint8_t i = 0; i < num; i++)
            temp_res += temp_array[i];
    }
    free(temp_array);
    return temp_res / num;
}

//@breif  实时更新线性加权滤波(压入数据同时滤波)
//@param  pbf    : 缓存器指针
//@param  pdata  : 压入数据的指针
//@param  pweight: 权重数组首地址
//@param  num    : 滤波数据长度
//@retval 返回线性加权滤波值
//@note   仅适用于float数据单元的buffer
float Buffer_Lin_Filter(buffer_t* pbf, const void* pdata, float* pweight, uint32_t num)
{
    assert_param(pbf);
    Buffer_Push(pbf, pdata);//存入最新的数据
    if(Buffer_UsedCount(pbf) <= num)//缓存区有效数据不足
        num = Buffer_UsedCount(pbf);//限制平均分母
    float temp_res = 0.0f;
    float* temp_array = NULL;
    temp_array = (float*)calloc(num, sizeof(float));//声明临时空间 同时清零
    if(temp_array)
    {
        Buffer_Gets(pbf, temp_array, 1, num);
        for(uint8_t i = 0; i < num; i++)
            temp_res += (temp_array[i] * pweight[i]);
    }
    free(temp_array);
    return temp_res / num;
}

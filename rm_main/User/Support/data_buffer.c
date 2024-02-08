#include "data_buffer.h"

//输入数据单元个数和占字节数，创建缓冲器
buffer_t* Buffer_Create(uint32_t num, uint32_t size)
{
	buffer_t* pubf = (buffer_t*)malloc(sizeof(buffer_t));
	void* pbuff = NULL;
	memset(pubf, 0, sizeof(buffer_t));
	if(pubf)
	{
		pubf->num = num;
		pubf->size = size;
		pbuff = calloc(num, size);//缓存器内存实体，同时清零
		assert_param(pbuff);
		pubf->pbuff = pbuff;
		return pubf;
	}
	return NULL;
}

//删除缓存器，释放内存
void Buffer_Delete(buffer_t** pbf)
{
	free((*pbf)->pbuff);
	free(*pbf);
	*pbf = NULL;//防止指针被进一步使用
}

//输入数据指针，投喂数据
void Buffer_Push(buffer_t* pbf, const void* pdata)
{
	assert_param(pbf);
	memcpy((uint8_t*)pbf->pbuff+pbf->write*pbf->size, pdata, pbf->size);
	if(++pbf->write >= pbf->num)
	{
		pbf->write = 0;
		pbf->isfull = 1;
	}
}

//获取k时刻的数据
void* Buffer_Pop(buffer_t* pbf, uint32_t k)
{
	void* res;
	if(k < pbf->write)//向前读取
		res = (uint8_t*)pbf->pbuff + (pbf->write-1-k)*pbf->size;
	else if(k < pbf->num)//折回尾部读取
		res = (uint8_t*)pbf->pbuff + ((pbf->num - 1) - (k - pbf->write)) * pbf->size;
	else//超出缓存容量
		res = NULL;
	return res;
}

//注意传进函数里的数组指针一定要提前声明好！
//取出缓存区若干数据，按照数据从新到旧排列成数组返回
//返回实际取出数量
uint32_t Buffer_Pops(buffer_t* pbf, void* pdata, uint32_t k, uint32_t num)
{
	assert_param(NULL != pbf && k <= Buffer_Used_Count(pbf)-1); //缓存器不存在 且 读取没有超容量
	void* ptr;
	uint32_t real_res_num = 0;
	for(uint32_t cnt = 0; cnt < num; ++cnt)  //cnt∈[0, num-1]，共（num-1-0+1=num）次自然循环
	{
		ptr = Buffer_Pop(pbf, k + cnt);
		if(ptr)
		{
			memcpy((uint8_t*)pdata+cnt*pbf->size, ptr, pbf->size);
			++real_res_num;
		}
		else break;
	}
	return real_res_num;
}

//获取缓存器中当前数据数量
uint32_t Buffer_Used_Count(buffer_t* pbf)
{
	uint32_t res;
	if(pbf == NULL)
		res = 0;
	else if(pbf->isfull)//缓存器已满
		res = pbf->num;
	else
		res = pbf->write;
	return res;
}

//清空缓存区
void Buffer_Flush(buffer_t* pbf)
{
	assert_param(pbf);
	memset(pbf->pbuff, 0, pbf->num * pbf->size);
	pbf->write = 0;
	pbf->isfull = 0;
}

//实时更新平均滤波
float Buffer_AVG_Filter(buffer_t* pbf, void* pdata, uint32_t num)
{
	assert_param(pbf && pdata && num);//缓存器存在 且数据量需求大于0
	Buffer_Push(pbf, pdata);//存入最新的数据
	if(Buffer_Used_Count(pbf) <= num)//缓存区有效数据不足
		num = Buffer_Used_Count(pbf);//限制平均分母
	
	float temp_res = 0.0f, *temp_array = NULL;
	temp_array = (float*)calloc(num, sizeof(float));//声明临时空间 同时清零
	if(temp_array)
	{
		Buffer_Pops(pbf, temp_array, 0, num);
		for(uint8_t i = 0; i < num; i++)
			temp_res += temp_array[i];
	}
	free(temp_array);
	return temp_res / num;
}

//线性加权滤波器（高阶低通滤波器）
//pweight:权重数组首地址
float Buffer_Lin_Filter(buffer_t* pbf, void* pdata, float* pweigth, uint32_t num)
{
	assert_param(pbf && pdata && num);//缓存器存在 且数据量需求大于0
	Buffer_Push(pbf, pdata);//存入最新的数据
	if(Buffer_Used_Count(pbf) <= num)//缓存区有效数据不足
		num = Buffer_Used_Count(pbf);//限制平均分母

	float temp_res = 0.0f, *temp_array = NULL;
	temp_array = (float*)calloc(num, sizeof(float));//声明临时空间 同时清零
	if(temp_array)
	{
		Buffer_Pops(pbf, temp_array, 0, num);
		for(uint8_t i = 0; i < num; i++)
			temp_res += (temp_array[i] * pweigth[i]);
	}
	free(temp_array);
	return temp_res / num;
}

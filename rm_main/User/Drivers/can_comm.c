#include "can_comm.h"
#include "Timu_driver.h"

FDCAN_TxHeaderTypeDef CAN_tx_message_standard;
FDCAN_RxHeaderTypeDef CAN1_rx_message, CAN2_rx_message, CAN3_rx_message;
uint8_t CAN1_rx_data[8], CAN2_rx_data[8], CAN3_rx_data[8];

//can总线初始化
void CAN_Comm_Init(void)
{
	FDCAN_FilterTypeDef can_filter;

	//can1过滤器设置
	can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
	can_filter.FilterIndex = 0;
	can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x001到0x003
	can_filter.FilterID1 = 0x001;
	can_filter.FilterID2 = 0x003;
	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);

	can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
	can_filter.FilterIndex = 1;
	can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x001到0x003
	can_filter.FilterID1 = 0x011;
	can_filter.FilterID2 = 0x013;
	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
	HAL_FDCAN_Start(&hfdcan1);
	
//	//can2过滤器设置
//	can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//	can_filter.FilterIndex = 0;
//	can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x011到0x013
//	can_filter.FilterID1 = 0x021;
//	can_filter.FilterID2 = 0x023;
//	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
//	HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);

//	can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//	can_filter.FilterIndex = 1;
//	can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x011到0x013
//	can_filter.FilterID1 = 0x011;
//	can_filter.FilterID2 = 0x013;
//	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
//	HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
//	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
//	HAL_FDCAN_Start(&hfdcan2);
//	
//	//can3过滤器设置
//	can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//	can_filter.FilterIndex = 0;
//	can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x001到0x003
//	can_filter.FilterID1 = 0x021;
//	can_filter.FilterID2 = 0x023;
//	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
//	HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);

//	can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//	can_filter.FilterIndex = 1;
//	can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x201到0x205
//	can_filter.FilterID1 = 0x011;
//	can_filter.FilterID2 = 0x013;
//	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
//	HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);
//	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
//	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
//	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
//	HAL_FDCAN_Start(&hfdcan3);
	
	//配置标准发送参数
	CAN_tx_message_standard.IdType = FDCAN_STANDARD_ID;
	CAN_tx_message_standard.TxFrameType = FDCAN_DATA_FRAME;
	CAN_tx_message_standard.DataLength = FDCAN_DLC_BYTES_8;
	CAN_tx_message_standard.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CAN_tx_message_standard.BitRateSwitch = FDCAN_BRS_ON;
	CAN_tx_message_standard.FDFormat = FDCAN_CLASSIC_CAN;
	CAN_tx_message_standard.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CAN_tx_message_standard.MessageMarker = 0;
}

//邮箱0接收回调函数，接收fdcan1，fdcan3数据
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if(hfdcan->Instance == FDCAN1)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_rx_message, CAN1_rx_data);
			Timu_Receive(&imu, CAN1_rx_message.Identifier, CAN1_rx_data);
			switch(CAN1_rx_message.Identifier)
			{
				default:break;
			}
		}
		else if(hfdcan->Instance == FDCAN3)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_rx_message, CAN3_rx_data);
			switch(CAN3_rx_message.Identifier)
			{
				default:break;
			}
		}
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	}
}

//邮箱1接收回调函数，接收fdcan2，fdcan3数据
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if(hfdcan->Instance == FDCAN2)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN2_rx_message, CAN2_rx_data);
			switch(CAN2_rx_message.Identifier)
			{
				default:break;
			}
		}
		else if(hfdcan->Instance == FDCAN3)
		{
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN3_rx_message, CAN3_rx_data);
			switch(CAN3_rx_message.Identifier)
			{
				default:break;
			}
		}
		HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	}
}

//can发送标准数据统一接口，提供给其它文件调用，8字节数据长度
void CAN1_Transmit(uint32_t tx_id, uint8_t* tx_data)
{
	CAN_tx_message_standard.Identifier = tx_id;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_tx_message_standard, tx_data);
}
void CAN2_Transmit(uint32_t tx_id, uint8_t* tx_data)
{
	CAN_tx_message_standard.Identifier = tx_id;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &CAN_tx_message_standard, tx_data);
}
void CAN3_Transmit(uint32_t tx_id, uint8_t* tx_data)
{
	CAN_tx_message_standard.Identifier = tx_id;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &CAN_tx_message_standard, tx_data);
}

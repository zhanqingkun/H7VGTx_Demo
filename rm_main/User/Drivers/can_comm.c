#include "can_comm.h"
#include "fdcan.h"

FDCAN_TxHeaderTypeDef tx_message;
FDCAN_RxHeaderTypeDef rx_fifo0_message, rx_fifo1_message;
uint8_t rx_fifo0_data[8], rx_fifo1_data[8];

/*
 * @brief  can总线初始化
 * @retval void
 * @note   设置过滤器，添加各驱动的初始化函数
 */
void can_comm_init(void)
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
    
//    //can2过滤器设置
//    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//    can_filter.FilterIndex = 0;
//    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x011到0x013
//    can_filter.FilterID1 = 0x021;
//    can_filter.FilterID2 = 0x023;
//    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
//    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);
//    
//    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//    can_filter.FilterIndex = 1;
//    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x011到0x013
//    can_filter.FilterID1 = 0x011;
//    can_filter.FilterID2 = 0x013;
//    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
//    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter);
//    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
//    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
//    HAL_FDCAN_Start(&hfdcan2);
//    
//    //can3过滤器设置
//    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//    can_filter.FilterIndex = 0;
//    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x001到0x003
//    can_filter.FilterID1 = 0x021;
//    can_filter.FilterID2 = 0x023;
//    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
//    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);
//    
//    can_filter.IdType = FDCAN_STANDARD_ID;//标准帧
//    can_filter.FilterIndex = 1;
//    can_filter.FilterType = FDCAN_FILTER_RANGE;//范围过滤 0x201到0x205
//    can_filter.FilterID1 = 0x011;
//    can_filter.FilterID2 = 0x013;
//    can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
//    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter);
//    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
//    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
//    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
//    HAL_FDCAN_Start(&hfdcan3);
    
    //配置标准发送参数
    tx_message.IdType = FDCAN_STANDARD_ID;
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.BitRateSwitch = FDCAN_BRS_ON;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_message.MessageMarker = 0;
    
    //各驱动初始化
}

/*
 * @brief  邮箱0接收回调函数
 * @retval void
 * @note   在其中添加各驱动的数据接收函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_fifo0_message, rx_fifo0_data);
        if (hfdcan->Instance == FDCAN1) {
            switch(rx_fifo0_message.Identifier) {
            case 0x01:
                ;
            default:
                break;
            }
        } else if (hfdcan->Instance == FDCAN2) {
            switch(rx_fifo0_message.Identifier) {
            case 0x01:
                ;
            default:
                break;
            }
        } else if (hfdcan->Instance == FDCAN3) {
            switch(rx_fifo0_message.Identifier) {
            case 0x01:
                ;
            default:
                break;
            }
        }
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}

/*
 * @brief  邮箱1接收回调函数
 * @retval void
 * @note   在其中添加各驱动的数据接收函数
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_fifo1_message, rx_fifo1_data);
        if (hfdcan->Instance == FDCAN1) {
            switch(rx_fifo1_message.Identifier) {
            case 0x01:
                ;
            default:
                break;
            }
        } else if (hfdcan->Instance == FDCAN2) {
            switch(rx_fifo1_message.Identifier) {
            case 0x01:
                ;
            default:
                break;
            }
        } else if (hfdcan->Instance == FDCAN3) {
            switch(rx_fifo1_message.Identifier) {
            case 0x01:
                ;
            default:
                break;
            }
        }
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
}

/*
 * @brief     can发送标准数据统一接口，提供给其它文件调用，8字节数据长度
 * @param[in] id: 帧id
 * @param[in] data: 数据指针
 * @retval    void
 */
void can1_std_transmit(uint32_t id, uint8_t *data)
{
    tx_message.Identifier = id;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_message, data);
}
void can2_std_transmit(uint32_t id, uint8_t *data)
{
    tx_message.Identifier = id;
//    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_message, data);
}
void can3_std_transmit(uint32_t id, uint8_t *data)
{
    tx_message.Identifier = id;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_message, data);
}

#include "bsp_can.h"
#include "stm32h7xx.h"
#include "fdcan.h"
#include "can_device.h"

FDCAN_TxHeaderTypeDef CAN_tx_message_standard;
FDCAN_RxHeaderTypeDef CAN1_rx_message, CAN2_rx_message, CAN3_rx_message;
uint8_t CAN1_rx_data[8], CAN2_rx_data[8], CAN3_rx_data[8];

FDCAN_RxHeaderTypeDef rx_fifo0_message, rx_fifo1_message;
uint8_t rx_fifo0_data[8], rx_fifo1_data[8];

void can_comm_init(void)
{
    FDCAN_FilterTypeDef can_filter;
    FDCAN_HandleTypeDef *can_handle;
    uint32_t filter_index;
    can_device_t *object = NULL;
    list_t *node = NULL;
    can_device_info_t *can_device_info;
    can_device_info = get_can_device_info();
    
    for (int can_periph = DEVICE_CAN1; can_periph < DEVICE_CAN_NUM; can_periph++) {//遍历can通道
        if (can_periph == DEVICE_CAN1)
            can_handle = &hfdcan1;
        else if (can_periph == DEVICE_CAN2)
            can_handle = &hfdcan2;
        else if (can_periph == DEVICE_CAN3)
            can_handle = &hfdcan3;
        for (int can_rxfifo = DEVICE_RXFIFO0; can_rxfifo < DEVICE_RXFIFO_NUM; can_rxfifo++) {//遍历邮箱
            if (can_rxfifo == DEVICE_RXFIFO0)
                can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;//通过过滤后给邮箱0
            else if (can_rxfifo == DEVICE_RXFIFO1)
                can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;//通过过滤后给邮箱1
            for (filter_index = 0, node = can_device_info->object_list[can_periph][can_rxfifo].next;//遍历某can通道某邮箱中的所有设备
                 node != &(can_device_info->object_list[can_periph][can_rxfifo]);
                 filter_index++, node = node->next) {
                object = list_entry(node, can_device_t, list);
                can_filter.IdType = object->id_type;
                can_filter.FilterIndex = filter_index;
                can_filter.FilterType = FDCAN_FILTER_MASK;//掩码模式
                can_filter.FilterID1 = object->id;
                can_filter.FilterID2 = 0xffffffff;
                HAL_FDCAN_ConfigFilter(can_handle, &can_filter);
            }
        }
        HAL_FDCAN_ConfigGlobalFilter(can_handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);//全局过滤器拒绝所有帧
        HAL_FDCAN_ActivateNotification(can_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能邮箱0新消息中断
        HAL_FDCAN_ActivateNotification(can_handle, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);//使能邮箱1新消息中断
        HAL_FDCAN_Start(can_handle);
    }
}


static void can_comm_rx_callback(FDCAN_HandleTypeDef *hfdcan, can_rxfifo_e can_rxfifo)
{
    int can_periph;
    list_t *node = NULL;
    can_device_t *object = NULL;
    can_device_info_t *can_device_info;
    can_device_info = get_can_device_info();
    FDCAN_RxHeaderTypeDef *rx_message;
    uint8_t *rx_data;

    if (hfdcan->Instance == FDCAN1)
        can_periph = DEVICE_CAN1;
    else if(hfdcan->Instance == FDCAN2)
        can_periph = DEVICE_CAN2;
    else if(hfdcan->Instance == FDCAN3)
        can_periph = DEVICE_CAN3;
    if (can_rxfifo == DEVICE_RXFIFO0) {
        rx_message = &rx_fifo0_message;
        rx_data = rx_fifo0_data;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, rx_message, rx_data);
    } else if (can_rxfifo == DEVICE_RXFIFO1) {
        rx_message = &rx_fifo1_message;
        rx_data = rx_fifo1_data;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, rx_message, rx_data);
    }

    for (node = can_device_info->object_list[can_periph][can_rxfifo].next;//遍历某can通道某邮箱中的所有设备
        node != &(can_device_info->object_list[can_periph][can_rxfifo]);
        node = node->next) {
        object = list_entry(node, can_device_t, list);
        if (object->id_type == rx_message->IdType && object->id == rx_message->Identifier) {
            if (object->device_type == DEVICE_DJI_MOTOR) {
                
            } else if (object->device_type == DEVICE_HT_MOTOR) {
                
            } else if (object->device_type == DEVICE_T_IMU) {
                
            }
        }
    }
    if (can_rxfifo == DEVICE_RXFIFO0)
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, rx_message, rx_data);
    else if (can_rxfifo == DEVICE_RXFIFO1)
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, rx_message, rx_data);
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		can_comm_rx_callback(hfdcan, DEVICE_RXFIFO0);
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
        can_comm_rx_callback(hfdcan, DEVICE_RXFIFO1);
	}
}


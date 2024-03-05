#include "can_device.h"
#include "stm32h7xx.h"

static can_device_info_t object_container = {{
    {
        {&(object_container.object_list[DEVICE_CAN1][DEVICE_RXFIFO1]), &(object_container.object_list[DEVICE_CAN1][DEVICE_RXFIFO1])}, 
        {&(object_container.object_list[DEVICE_CAN1][DEVICE_RXFIFO2]), &(object_container.object_list[DEVICE_CAN1][DEVICE_RXFIFO2])}
    }, 
    {
        {&(object_container.object_list[DEVICE_CAN2][DEVICE_RXFIFO1]), &(object_container.object_list[DEVICE_CAN2][DEVICE_RXFIFO1])}, 
        {&(object_container.object_list[DEVICE_CAN2][DEVICE_RXFIFO2]), &(object_container.object_list[DEVICE_CAN2][DEVICE_RXFIFO2])}
    },
    {
        {&(object_container.object_list[DEVICE_CAN3][DEVICE_RXFIFO1]), &(object_container.object_list[DEVICE_CAN3][DEVICE_RXFIFO1])}, 
        {&(object_container.object_list[DEVICE_CAN3][DEVICE_RXFIFO2]), &(object_container.object_list[DEVICE_CAN3][DEVICE_RXFIFO2])}
    }
}};

can_device_info_t *get_can_device_info(void)
{
    return &object_container;
}

void can_device_init(can_device_t *object,
                     can_device_type_e device_type,
                     can_periph_e can_periph,
                     can_rxfifo_e can_rxfifo,
                     uint32_t id,
                     uint32_t id_type)
{
    assert_param(object);
    assert_param(device_type < DEVICE_TYPE_NUM);
    assert_param(can_periph < DEVICE_CAN_NUM);
    assert_param(can_rxfifo < DEVICE_RXFIFO_NUM);
    assert_param((id_type == FDCAN_STANDARD_ID && id <= 0x7ff) || 
                 (id_type == FDCAN_EXTENDED_ID && id <= 0x1fffffff));
    
    object->device_type = device_type;
    object->can_periph = can_periph;
    object->id = id;
    object->id_type = id_type;
    
    list_add(&(object->list), &(object_container.object_list[can_periph][can_rxfifo]));
}

can_device_t *can_device_find(can_device_type_e device_type,
                              can_periph_e can_periph,
                              uint32_t id)
{
    can_device_t *object = NULL;
    list_t *node = NULL;
    assert_param(device_type < DEVICE_TYPE_NUM);
    assert_param(can_periph < DEVICE_CAN_NUM);
    for (int i = DEVICE_RXFIFO1; i < DEVICE_RXFIFO_NUM; i++) {
        for (node = object_container.object_list[can_periph][i].next;
             node != &(object_container.object_list[can_periph][i]);
             node = node->next) {
            object = list_entry(node, can_device_t, list);
            if (object->device_type == device_type &&
                object->can_periph == can_periph &&
                object->id == id)
                return object;
        }
    }
    return NULL;
}

void can_device_detach(can_device_t *object)
{
    assert_param(object);
    object->device_type = DEVICE_UNKNOW;
    list_del(&(object->list));
}

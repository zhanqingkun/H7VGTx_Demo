#ifndef __CAN_DEVICE_H
#define __CAN_DEVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"
#include "data_list.h"

typedef enum
{
    DEVICE_CAN1 = 0,
    DEVICE_CAN2,
    DEVICE_CAN3,
    DEVICE_CAN_NUM
} can_periph_e;

typedef enum
{
    DEVICE_RXFIFO0 = 0,
    DEVICE_RXFIFO1,
    DEVICE_RXFIFO_NUM
} can_rxfifo_e;

typedef enum
{
    DEVICE_INIT = 0,
    DEVICE_DJI_MOTOR,
    DEVICE_HT_MOTOR,
    DEVICE_T_IMU,
    DEVICE_UNKNOW,
    DEVICE_TYPE_NUM
} can_device_type_e;

typedef struct
{
    list_t list;
    can_device_type_e device_type;
    can_periph_e can_periph;
    can_rxfifo_e can_rxfifo;
    uint32_t id;
    uint32_t id_type;
} can_device_t;

typedef struct
{
    list_t object_list[DEVICE_CAN_NUM][DEVICE_RXFIFO_NUM];
} can_device_info_t;

can_device_info_t *get_can_device_info(void);
void can_device_init(can_device_t *object,
                     can_device_type_e device_type,
                     can_periph_e can_periph,
                     can_rxfifo_e can_rxfifo,
                     uint32_t id,
                     uint32_t id_type);
can_device_t *can_device_find(can_device_type_e device_type,
                              can_periph_e can_periph,
                              uint32_t id);
void can_device_detach(can_device_t *object);

#ifdef __cplusplus
}
#endif

#endif

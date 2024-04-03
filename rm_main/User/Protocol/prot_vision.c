#include "prot_vision.h"
#include "string.h"

vision_t vision;

void vision_get_data(uint8_t *data)
{
    memcpy(vision.rx[0].buff, data, VISION_DATA_LEN);
    
    
}

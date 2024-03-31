#ifndef __BSP_COVER_H__
#define __BSP_COVER_H__

typedef enum
{
    HOUSE_MODE_OPEN = 0,
    HOUSE_MODE_CLOSE = 1,
    HOUSE_MODE_PROTECT = 2
} house_mode_e;

extern house_mode_e house_mode;
void house_init(void);
void house_control(void);

#endif

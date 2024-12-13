#ifndef LAMP_H
#define LAMP_H
#include <stdint.h>

typedef enum
{
    LIGHT_CONTROL = 1,
    SOUND_CONTROL,
    MANUAL_CONTROL,
    BREATHING_LIGHT,
    EVIL_BIG_MOUSE,
} control_mode_e;

typedef struct
{
    control_mode_e control_mode;
    uint16_t manual_intensity;
} Lamp_data_t;

void lamp_init();
void lamp_task();

#endif // !LAMP_H
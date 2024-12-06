#ifndef LAMP_H
#define LAMP_H

typedef enum
{
    LIGHT_CONTROL = 0,
    SOUND_CONTROL = 1,
} control_mode_e;

void lamp_init();
void lamp_task();

#endif // !LAMP_H
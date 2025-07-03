#include "buzzer.h"

// TODO: 添加不同频率的反馈

void buzzer_init()
{
    gpio_init(BEEP_ENABLE, GPO, 0, GPO_PUSH_PULL);
}

void buzzer_on()
{
    gpio_set_level(BEEP_ENABLE, 1);
}

void buzzer_off()
{
    gpio_set_level(BEEP_ENABLE, 1);
}

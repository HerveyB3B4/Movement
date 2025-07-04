#include "buzzer.h"
#include "pin.h"
#include "zf_driver_gpio.h"

static BUZZER_INFO buzzer_info;
static BUZZER_STATUS_e buzzer_status;
static uint8[2] buzzer_count;

void buzzer_init()
{
    gpio_init(BEEP_ENABLE, GPO, 0, GPO_PUSH_PULL);
}

void buzzer_set(uint8 on, uint8 pause, uint8 count)
{
    buzzer_info.buzzer_on_interrupts = on;
    buzzer_info.buzzer_pause_interrupts = pause;
    buzzer_info.buzzer_counts = count;
}

void buzzer_on()
{
    buzzer_status = BUZZER_ON;
    buzzer_count = {0, 0};
    gpio_set_level(BEEP_ENABLE, BUZZER_ON);
}

void buzzer_off()
{
    buzzer_status = BUZZER_OFF;
    buzzer_count = {0, 0};
    gpio_set_level(BEEP_ENABLE, BUZZER_OFF);
}

void buzzer_handler()
{
    buzzer_count[0]++;
    switch (buzzer_status)
    {
    case BUZZER_ON:
        if (buzzer_count[0] == buzzer_info.buzzer_on_interrupts)
        {
            buzzer_count[0] = 0;
            buzzer_count[1]++;
            buzzer_status = BUZZER_PAUSE;
        }
        break;
    case BUZZER_PAUSE:
        if (buzzer_count[0] == buzzer_info.buzzer_pause_interrupts)
        {
            buzzer_count[0] = 0;
            buzzer_status = BUZZER_ON;
        }
        break;
    }
    if (buzzer_count[1] == buzzer_info.buzzer_counts)
    {
        buzzer_count[1] = 0;
        buzzer_off();
    }
}
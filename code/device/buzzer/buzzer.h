#ifndef _BEEP_h_
#define _BEEP_h_

#include "zf_common_headfile.h"

#define BEEP_ENABLE BUZZER_PIN
#define BEEP_PWM BUZZER_PWM

typedef struct {
    uint8 buzzer_on_interrupts, buzzer_pause_interrupts, buzzer_counts;
} BUZZER_INFO;

typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ON = 1,
    BUZZER_PAUSE, // TODO: 测试能不能用 -1
} BUZZER_STATUS_e;

void buzzer_init(void);
void buzzer_set(uint8, uint8, uint8);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_handler(void);

#endif

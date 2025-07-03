#ifndef _BEEP_h_
#define _BEEP_h_

#include "zf_common_headfile.h"

#define BEEP_ENABLE BUZZER_PIN
#define BEEP_PWM BUZZER_PWM

void BEEF_init(void);
void BEEF_on(void);
void BEEF_off(void);

#endif

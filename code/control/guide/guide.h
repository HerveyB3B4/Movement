#ifndef _CONTROL_GUIDE_H
#define _CONTROL_GUIDE_H

#include "zf_common_headfile.h"
#include "control.h"

#define GUIDE_CONTANT_VEL -2000

int16 get_guide_target_vel(void);
int16 get_guide_target_turn(void);

void guide_set_target_vel(int16 target_vel);
void guide_set_target_turn(int16 target_turn);

void guide_receiver(struct Control_Target *control_target);
void guide_to_target(struct Control_Target *control_target);

#endif
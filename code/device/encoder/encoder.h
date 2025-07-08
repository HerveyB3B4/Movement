#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_headfile.h"
//===========================================encoder==============================================

struct Velocity_Motor;

void encoder_init();
void encoder_get_momentum(struct Velocity_Motor *motorVelocity);
int16 encoder_get_bottom();

#endif
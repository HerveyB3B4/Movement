#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_headfile.h"
//===========================================encoder==============================================

struct Velocity_Motor;

void encoder_init();
void get_momentum_encoder(struct Velocity_Motor *motorVelocity);
void get_bottom_encoder(struct Velocity_Motor *motorVelocity);

#endif
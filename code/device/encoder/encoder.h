#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_headfile.h"
#include "pin.h"

//===========================================pin==================================================
#define BOTTOM_ENCODER ENCODER_BOTTOM
#define BOTTOM_ENCODER_PIN0 ENCODER_BOTTOM_PIN0
#define BOTTOM_ENCODER_PIN1 ENCODER_BOTTOM_PIN1

//===========================================encoder==============================================

struct Velocity_Motor;

void encoder_init();
void encoder_get_momentum(struct Velocity_Motor *motorVelocity);
int16 encoder_get_bottom();

#endif
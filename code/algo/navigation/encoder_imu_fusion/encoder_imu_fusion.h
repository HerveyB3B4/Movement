#ifndef _ENCODER_IMU_FUSION_H_
#define _ENCODER_IMU_FUSION_H_

#include "zf_common_headfile.h"

typedef struct
{
    float x;
    float y;
} fusion_nav_pos;

void fusion_nav_init();
void fusion_nav_update(void);

#endif
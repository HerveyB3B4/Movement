#ifndef _STATE_MACHINE_H
#define _STATE_MACHINE_H

#include "zf_common_headfile.h"

typedef enum
{
    STATE_SEARCHING,   // 状态：寻找目标
    STATE_APPROACHING, // 状态：接近目标
    STATE_CIRCLING,    // 状态：环绕目标
} Run_State;

#endif

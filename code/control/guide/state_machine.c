#include "state_machine.h"

static Run_State curr_state;

void state_machine_init()
{
    curr_state = STATE_SEARCHING;
}
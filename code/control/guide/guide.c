#include "guide.h"
#include "image.h"
#include "control.h"

static int16 guide_target_vel = 0;
static int16 guide_target_turn = 0;

static int16 last_target_error = 0;
static bool target_lost = false;
static int16 search_turn_value = 1000; // 目标丢失时的转向强度

void guide_set_target_vel(int16 target_vel)
{
    restrictValueI(&target_vel, -9999, 9999);

    guide_target_vel = target_vel;
}

void guide_set_target_turn(int16 target_turn)
{
    // TODO 限幅
    guide_target_turn = target_turn;
}

int16 get_guide_target_vel(void)
{
    return guide_target_vel;
}

int16 get_guide_target_turn(void)
{
    return guide_target_turn;
}

void guide_receiver(struct Control_Target *control_target)
{
    // 直接更新目标
    control_target->bottom_vel = (float)guide_target_vel;
    control_target->turn_err = guide_target_turn;
}

void guide_to_target(struct Control_Target *control_target)
{
    // if (get_img_target_distance() != 0 && guide_target_vel != 0)
    // {
    //     control_target->bottom_vel = -(float)guide_target_vel;
    // }

    int16 current_error = get_img_target_error();

    if (get_img_target_distance() > 0)
    {
        // 目标有效
        target_lost = false;
        last_target_error = current_error;
        control_target->turn_err = current_error;
    }
    else
    {
        if (!target_lost)
        {
            target_lost = true;
        }

        if (last_target_error > 0)
        {
            control_target->turn_err = search_turn_value;
        }
        else if (last_target_error < 0)
        {
            control_target->turn_err = -search_turn_value;
        }
        else
        {
            control_target->turn_err = 0;
        }
    }
}

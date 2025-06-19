#include "guide.h"
#include "image.h"

static int16 guide_target_vel = 0;
static int16 guide_target_turn = 0;

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

    control_target->turn_err = get_img_target_error();
}

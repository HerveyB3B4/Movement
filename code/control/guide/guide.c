#include "guide.h"
#include "image.h"
#include "control.h"
#include "distance.h"
#include "detection.h"
#include "pid.h"

static int16 guide_target_vel = 0;
static int16 guide_target_turn = 0;

static int16 last_target_error = 0;
static bool target_lost = false;
static int16 search_turn_value = 1000; // 目标丢失时的转向强度

void guide_set_target_vel(int16 target_vel)
{
    restrictValueI(&target_vel, -9999, 9999);

    guide_target_vel = -target_vel;
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

void guide_position_pid(struct Control_Target *control_target, Point *target)
{
    control_target->bottom_vel = PID_calc_Position(&bottom_position_PID,
                                                   distance_reckon(target->x, target->y, 0),
                                                   10.0f);
}

void guide_to_target(struct Control_Target *control_target)
{
    // control_target->bottom_vel = (float)guide_target_vel;
    // control_target->turn_err = get_img_target_error();
    Point target_point = get_target_point();
    guide_position_pid(control_target, &target_point);
}

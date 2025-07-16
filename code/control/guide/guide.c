#include "guide.h"
#include "image.h"
#include "control.h"
#include "distance.h"
#include "detection.h"
#include "pid.h"
#include "receiver.h"

static int32 guide_target_vel = 0;
static int32 guide_target_turn = 0;

static int16 search_turn_value = 1500; // 目标丢失时的转向强度

void guide_set_target_vel(int32 target_vel)
{
    restrictValueI(&target_vel, -9999, 9999);
    guide_target_vel = -target_vel;
}

void guide_set_target_turn(int32 target_turn)
{
    guide_target_turn = target_turn;
    // guide_target_turn = PID_calc(&turn_PID, (float)target_turn);
    // restrictValueI(&guide_target_turn, -5000, 5000); // 限制转向输出
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
    static uint32 cnt = 0;
    cnt++;

    // if (cnt >= 2000)
    // {
    //     control_target->bottom_vel = 20;
    // }
    // if (cnt >= 3000)
    // {
    //     control_target->bottom_vel = 20;
    //     control_target->turn_err = 50;
    //     // control_target->buckling_front = 0.5 * (g_vel_motor.bottom - control_target->bottom_vel);
    // }
    // if (cnt >= 4000)
    // {
    //     control_target->bottom_vel = 20;
    //     control_target->turn_err = 100;
    //     // control_target->buckling_front = (g_vel_motor.bottom - control_target->bottom_vel);
    // }

    // if (cnt >= 6000)
    // {
    //     control_target->bottom_vel = 20;
    //     control_target->turn_err = 200;
    //     // control_target->buckling_front = 1.5 * (g_vel_motor.bottom - control_target->bottom_vel);
    // }
    // if (cnt >= 7000)
    // {
    //     control_target->bottom_vel = 20;
    //     control_target->turn_err = 250;
    // }
    // if (cnt >= 8000)
    // {
    //     control_target->bottom_vel = 20;
    //     control_target->turn_err = 0;
    // }
    // if (cnt >= 9000)

    // {
    //     control_target->bottom_vel = 0;
    // }

    // else if (cnt <= 6000)
    // {
    //     control_target->bottom_vel = 30;
    // }
    // else
    // {
    //     cnt = 0; // 重置计数器，开始新的周期
    // }
    control_target->bottom_vel = (float)guide_target_vel;
    control_target->turn_err = (float)guide_target_turn;
}

void guide_position_pid(struct Control_Target *control_target, Point *target)
{
    control_target->bottom_vel = PID_calc_Position(&bottom_position_PID,
                                                   distance_reckon(target->x, target->y, 0),
                                                   10.0f);
}

void guide_to_target(struct Control_Target *control_target)
{
    control_target->bottom_vel = (float)guide_target_vel;
    // control_target->turn_err = get_img_target_error();
    // control_target->bottom_vel = (float)guide_target_vel;
    // control_target->turn_err = get_img_target_error();
    // Point target_point = get_target_point();
    // guide_position_pid(control_target, &target_point);
    // AI:
    // guide_to_target 函数现在的作用就是将状态机计算出的速度和转向值应用到控制目标上
    // control_target->bottom_vel = (float)get_guide_target_vel();
    // control_target->turn_err = (float)get_guide_target_turn();
}

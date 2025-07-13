#include "menu_input.h"
#include "attitude.h"
#include "control.h"
#include "guide.h"

struct Menu_Manual_Param g_menu_manual_param;

void menu_manual_param_init()
{
    memset(&g_menu_manual_param, 0, sizeof(struct Menu_Manual_Param));
}

void menu_get_params(
    struct EulerAngle *euler_angle_bias,
    struct Control_Time *control_time,
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Control_Motion_Manual_Parmas *control_motion_params)
{
    // euler angle
    euler_angle_bias->roll = g_menu_manual_param.mechanicalRollAngle * 0.1f;
    euler_angle_bias->pitch = g_menu_manual_param.mechanicalPitchAngle * 0.1f;
    euler_angle_bias->yaw = g_menu_manual_param.mechanicalYawAngle * 0.1f;

    // pid
    for (uint8 i = 0; i < 3; i++)
    {
        control_motion_params->bottom_angle_velocity_parameter[i] =
            g_menu_manual_param.bottom_angle_velocity_parameter[i];
        control_motion_params->bottom_angle_parameter[i] =
            g_menu_manual_param.bottom_angle_parameter[i];
        control_motion_params->bottom_velocity_parameter[i] =
            g_menu_manual_param.bottom_velocity_parameter[i];
        control_motion_params->bottom_position_parameter[i] =
            g_menu_manual_param.bottom_position_parameter[i];

        control_motion_params->side_angle_velocity_parameter[i] =
            g_menu_manual_param.side_angle_velocity_parameter[i];
        control_motion_params->side_angle_parameter[i] =
            g_menu_manual_param.side_angle_parameter[i];
        control_motion_params->side_velocity_parameter[i] =
            g_menu_manual_param.side_velocity_parameter[i];

        control_motion_params->turn_angle_velocity_parameter[i] =
            g_menu_manual_param.turn_angle_velocity_parameter[i];
        control_motion_params->turn_error_parameter[i] =
            g_menu_manual_param.turn_error_parameter[i];
        control_motion_params->turn_velocity_parameter[i] =
            g_menu_manual_param.turn_velocity_parameter[i];
        control_motion_params->turn_angle_parameter[i] =
            g_menu_manual_param.turn_angle_parameter[i];
    }

    // 传递PID极性参数
    // if (g_menu_manual_param.bottom_velocity_polarity == 0)
    // {
    //     control_motion_params->bottom_velocity_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->bottom_velocity_polarity = -1;
    // }
    // if (g_menu_manual_param.bottom_angle_velocity_polarity == 0)
    // {
    //     control_motion_params->bottom_angle_velocity_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->bottom_angle_velocity_polarity = -1;
    // }
    // if (g_menu_manual_param.bottom_angle_polarity == 0)
    // {
    //     control_motion_params->bottom_angle_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->bottom_angle_polarity = -1;
    // }
    // if (g_menu_manual_param.side_angle_velocity_polarity == 0)
    // {
    //     control_motion_params->side_angle_velocity_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->side_angle_velocity_polarity = -1;
    // }
    // if (g_menu_manual_param.side_angle_polarity == 0)
    // {
    //     control_motion_params->side_angle_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->side_angle_polarity = -1;
    // }
    // if (g_menu_manual_param.side_velocity_polarity == 0)
    // {
    //     control_motion_params->side_velocity_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->side_velocity_polarity = -1;
    // }
    // if (g_menu_manual_param.turn_angle_polarity == 0)
    // {
    //     control_motion_params->turn_angle_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->turn_angle_polarity = -1;
    // }
    // if (g_menu_manual_param.turn_velocity_polarity == 0)
    // {
    //     control_motion_params->turn_velocity_polarity = 1;
    // }
    // else
    // {
    //     control_motion_params->turn_velocity_polarity = -1;
    // }

    // control_motion_params->bottom_velocity_polarity =
    //     g_menu_manual_param.bottom_velocity_polarity;
    // control_motion_params->bottom_angle_velocity_polarity =
    //     g_menu_manual_param.bottom_angle_velocity_polarity;
    // control_motion_params->bottom_angle_polarity =
    //     g_menu_manual_param.bottom_angle_polarity;
    // control_motion_params->side_angle_velocity_polarity =
    //     g_menu_manual_param.side_angle_velocity_polarity;
    // control_motion_params->side_angle_polarity =
    //     g_menu_manual_param.side_angle_polarity;
    // control_motion_params->side_velocity_polarity =
    //     g_menu_manual_param.side_velocity_polarity;
    // control_motion_params->turn_angle_polarity =
    //     g_menu_manual_param.turn_angle_polarity;
    // control_motion_params->turn_velocity_polarity =
    //     g_menu_manual_param.turn_velocity_polarity;

    // turn
    // control_turn_params->buckling_side_state =
    //     (float)g_menu_manual_param.buckingTurnCoefficient * 0.01f;
    // control_turn_params->turn_gain_coefficient =
    //     (float)g_menu_manual_param.turnGainCoefficient * 0.01f;
    // control_turn_params->buckling_front_coefficientV =
    //     g_menu_manual_param.bucklingFrontCoefficientV;
    // control_turn_params->buckling_front_coefficientT =
    //     g_menu_manual_param.bucklingFrontCoefficientT;

    control_time->turn[0] = g_menu_manual_param.TurnControlTimeParameter[0]; // TAV
    control_time->turn[1] = g_menu_manual_param.TurnControlTimeParameter[1]; // TE
    control_time->turn[2] = g_menu_manual_param.TurnControlTimeParameter[2]; // TV
    control_time->turn[3] = g_menu_manual_param.TurnControlTimeParameter[3]; // TA

    control_time->bottom[0] = g_menu_manual_param.FrontControlTimeParameter[0]; // FAV
    control_time->bottom[1] = g_menu_manual_param.FrontControlTimeParameter[1]; // FA
    control_time->bottom[2] = g_menu_manual_param.FrontControlTimeParameter[2]; // FV

    control_time->side[0] = g_menu_manual_param.SideControlTimeParameter[0]; // SAV
    control_time->side[1] = g_menu_manual_param.SideControlTimeParameter[1]; // SA
    control_time->side[2] = g_menu_manual_param.SideControlTimeParameter[2]; // SV

    guide_set_target_vel(g_menu_manual_param.bottom_velocity);
    guide_set_target_turn(g_menu_manual_param.turn_target * 0.01f);
    side_set_internal_diff(g_menu_manual_param.side_internal_diff);

    control_turn_params->buckling_side_state =
        g_menu_manual_param.buckling_side_state * 0.1f;
    control_turn_params->buckling_side_dynamic =
        g_menu_manual_param.buckling_side_dynamic * 0.001f;
    control_turn_params->buckling_front_coefficient =
        g_menu_manual_param.buckling_front_coefficient * 0.1f;

    g_angle_limit = g_menu_manual_param.angle_limit * 0.1f;
}
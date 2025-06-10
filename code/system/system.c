#include "system.h"
#include "attitude.h"
#include "control.h"
#include "lcd.h"
#include "menu.h"
#include "menu_input.h"
#include "small_driver_uart_control.h"
#include "velocity.h"
#include "zf_common_headfile.h"
#include "key.h"
#include "receiver.h"

RunState_t runState;

void system_init()
{
    // uart_init(UART_2, 115200, UART2_TX_P10_5, UART2_RX_P10_6);
    // init motor
    motor_init();
    // 串口初始化放到这里不知道会不会好点
    small_driver_uart_init();
    // init encoder
    encoder_init();
    // init lcd
    lcd_init();
    // uart_init(UART_1, 115200, UART1_RX_P20_9, UART1_TX_P20_10);
    // uart_write_string(UART_1, "test");
    // init camera
    mt9v03x_init();
    // mt9v03x2_init();
    // receiver_init();
    // init wireless uart
    // wireless_init();

    // init key
    key_init_rewrite(KEY_NUM);
    pit_ms_init(CCU60_CH1, KEY_UPDATE_T);
    pit_enable(CCU60_CH1); // 使能按键中断
    // menu_param
    menu_manual_param_init();

    // velocity
    velocity_init(&g_vel_motor);
    pit_ms_init(CCU60_CH0, VELOCITY_UPDATE_T);
    pit_enable(CCU60_CH0); // 使能速度中断

    control_manual_param_init();

    // read eeprom
    Read_EEPROM();

    // init imu
    imu_init();
    // init attitude
    attitude_init();
    pit_ms_init(CCU61_CH1, ATTITUDE_UPDATE_T);
    pit_enable(CCU61_CH1); // 使能姿态中断

    // menu
    MainMenu_Set();

    // menu_params
    menu_get_params(&g_euler_angle_bias, &g_control_time,
                    &g_control_turn_manual_params, &g_control_motion_params);

    // control init
    control_init(&g_control_motion_params);
    // start to balance
    pit_ms_init(CCU61_CH0, CONTROL_UPDATE_T);
}

void system_attitude_timer(
    struct Control_Turn_Manual_Params *control_turn_params,
    struct Control_Target *control_target,
    struct Velocity_Motor *vel_motor,
    struct EulerAngle *euler_angle)
{
    static uint8 imuCnt = 0;
    imuCnt++;
    if (imuCnt >= 2)
    {
        imuCnt = 0;
        g_attitude_cal_flag = 1;
        attitude_cal_amend(control_turn_params, control_target, vel_motor,
                           euler_angle);
    }
    else
    {
        g_attitude_cal_flag = 0;
    }
}

void bottom_control_timer(struct Control_Time *control_time,
                          struct Control_Flag *control_flag,
                          struct Control_Target *control_target,
                          struct Velocity_Motor *vel_motor,
                          struct EulerAngle *euler_angle_bias,
                          struct Control_Motion_Manual_Parmas *control_motion_params)
{
    uint32 frontAngleTime = control_time->bottom[0];
    uint32 frontAngleVelocityTime = control_time->bottom[1];
    uint32 frontVelocityTime = control_time->bottom[2];

    control_flag->bottom_angle_cnt++;
    control_flag->bottom_angle_vel_cnt++;
    control_flag->bottom_vel_cnt++;
    if (control_flag->bottom_angle_cnt >= frontAngleTime)
    { // 20ms
        control_flag->bottom_angle = 1;
        control_flag->bottom_angle_cnt = 0;
    }
    else
    {
        control_flag->bottom_angle = 0;
    }
    if (control_flag->bottom_angle_vel_cnt >=
        frontAngleVelocityTime)
    { // 10ms
        control_flag->bottom_angle_vel = 1;
        control_flag->bottom_angle_vel_cnt = 0;
    }
    else
    {
        control_flag->bottom_angle_vel = 0;
    }
    if (control_flag->bottom_vel_cnt >= frontVelocityTime)
    { // 2ms
        control_flag->bottom_vel = 1;
        control_flag->bottom_vel_cnt = 0;
    }
    else
    {
        control_flag->bottom_vel = 0;
    }
    control_bottom_balance(control_target, control_flag, vel_motor,
                           euler_angle_bias, control_motion_params);
}

void side_control_timer(struct Control_Time *control_time,
                        struct Control_Flag *control_flag,
                        struct Control_Target *control_target,
                        struct Control_Turn_Manual_Params *control_turn_params,
                        struct Velocity_Motor *vel_motor,
                        struct EulerAngle *euler_angle_bias,
                        struct Control_Motion_Manual_Parmas *control_motion_params)
{
    uint32 sideAngleTime = control_time->side[0];
    uint32 sideAngleVelocityTime = control_time->side[1];
    uint32 sideVelocityTime = control_time->side[2];
    control_flag->side_vel_cnt++;
    control_flag->side_angle_cnt++;
    control_flag->side_angle_vel_cnt++;
    if (control_flag->side_vel_cnt >= sideVelocityTime)
    {
        control_flag->side_vel = 1;
        control_flag->side_vel_cnt = 0;
    }
    else
    {
        control_flag->side_vel = 0;
    }
    if (control_flag->side_angle_cnt >= sideAngleTime)
    {
        control_flag->side_angle = 1;
        control_flag->side_angle_cnt = 0;
    }
    else
    {
        control_flag->side_angle = 0;
    }
    if (control_flag->side_angle_vel_cnt >= sideAngleVelocityTime)
    {
        control_flag->side_angle_vel = 1;
        control_flag->side_angle_vel_cnt = 0;
    }
    else
    {
        control_flag->side_angle_vel = 0;
    }
    control_side_balance(control_target, control_flag, control_turn_params,
                         vel_motor, euler_angle_bias, control_motion_params);
}

void turn_control_timer(struct Control_Time *control_time,
                        struct Control_Flag *control_flag,
                        struct Control_Target *control_target,
                        struct Velocity_Motor *vel_motor,
                        struct EulerAngle *euler_angle_bias)
{
    uint32 turnAngleTime = control_time->turn[0];
    uint32 turnVelocityTime = control_time->turn[1];
    uint32 turnCurvatureTime = turnAngleTime * 4;
    control_flag->turn++;
    control_flag->turnAngleCount++;
    control_flag->turnAngleDiffVelocityCount++;
    if (control_flag->turn >= turnCurvatureTime)
    {
        control_flag->turn_angle = 1;
        control_flag->turn = 0;
    }
    else
    {
        control_flag->turn_angle = 0;
    }
    if (control_flag->turnAngleCount >= turnAngleTime)
    {
        control_flag->turn_angle = 1;
        control_flag->turnAngleCount = 0;
    }
    else
    {
        control_flag->turn_angle = 0;
    }
    if (control_flag->turnAngleDiffVelocityCount >= turnVelocityTime)
    {
        control_flag->turnAngleDiffVelocity = 1;
        control_flag->turnAngleDiffVelocityCount = 0;
    }
    else
    {
        control_flag->turnAngleDiffVelocity = 0;
    }
    // control_turn_balance();
}

void system_set_runstate(RunState_t state)
{
    // 根据不同的车辆状态执行不同的控制操作
    switch (state)
    {
    case CAR_STOP:
        runState = CAR_STOP;

        pit_disable(CCU61_CH0); // 失能控制中断
        pit_enable(CCU60_CH1);  // 使能按键中断

        stop_bottom_motor();
        stop_momentum_motor();
        zf_log(0, "shutdown");
        break;
    case CAR_RUNNING:
        runState = CAR_RUNNING;

        // 在切换到运行状态时重置控制目标和标志
        control_reset(&g_control_target);

        // pit_enable(CCU61_CH0);  // 使能控制中断
        pit_disable(CCU60_CH1); // 失能按键中断
        break;
    }
}

void system_control()
{
    if (g_control_bottom_flag != 0)
    {
        g_control_target.bottom_vel = g_received_vel;
        bottom_control_timer(&g_control_time, &g_control_flag,
                             &g_control_target, &g_vel_motor,
                             &g_euler_angle_bias,
                             &g_control_motion_params);
    }

    // turnControlTimer();
    if (g_control_side_flag != 0)
    {
        side_control_timer(&g_control_time, &g_control_flag,
                           &g_control_target,
                           &g_control_turn_manual_params, &g_vel_motor,
                           &g_euler_angle_bias,
                           &g_control_motion_params);
    }

    control_shutdown(&g_control_target, &g_euler_angle_bias);
}
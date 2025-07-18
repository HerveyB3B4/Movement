/*********************************************************************************************************************
 * TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK
 *接口的第三方开源库 Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC377 开源库的一部分
 *
 * TC377 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即
 *GNU通用公共许可证）的条款 即 GPL 的第3版（即
 *GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt
 *文件中 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu0_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.10.2
 * 适用平台          TC377TP
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-11-03       pudding            first version
 ********************************************************************************************************************/
#include "Attitude.h"
#include "control.h"
#include "menu.h"
#include "motor.h"
#include "image.h"
#include "velocity.h"
#include "zf_common_headfile.h"
#include "Mahony.h"
#include "control.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

int core0_main(void)
{
    clock_init();

    system_init();

    cpu_wait_event_ready();

    g_exit_menu_flag = 1;
    system_start();

    system_set_runstate(CAR_RUNNING);
    while (TRUE)
    {
        // printf("%f,%f,%f\n",
        //        g_control_target.bottom_angle,
        //        g_control_target.bottom_angle_vel,
        //        PITCH);
        // printf("%f\n", g_control_target.side_angle);
        // printf("%f\n", get_cpu_freq());
        // printf("%f\n", ROLL_VEL);
        // printf("%f,%f\n", twoKpDef, twoKiDef);
        // lcd_show_float(0, 0, side_angle_PID.Kp, 3, 3);
        // lcd_show_float(0, 1, side_angle_PID.Ki, 3, 3);
        // lcd_show_float(6, 0, side_velocity_PID.Kp, 3, 3);
        // lcd_show_float(6, 1, side_velocity_PID.Ki, 3, 3);
        // lcd_show_float(0, 1, ROLL, 3, 6);
        // lcd_show_uint(0, 0, mahony_cnt, 5);
        // img_handler(g_show_run_param_flag);
        // img_handler(g_show_run_param_flag);
        // lcd_show_int(0, 0, get_momentum_diff(), 5);
        // lcd_show_int(0, 1, get_side_duty(), 5);
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************

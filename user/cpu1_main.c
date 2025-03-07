/*********************************************************************************************************************
 * TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC377 开源库的一部分
 *
 * TC377 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
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
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu1_main
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

#include "zf_common_headfile.h"
#include "attitude.h"
#include "control.h"
#include "menu.h"
#include "system.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
void core1_main(void)
{
    disable_Watchdog();         // 关闭看门狗
    interrupt_global_enable(0); // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    system_init();
    runState = CAR_READY;

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready(); // 等待所有核心初始化完毕
    g_exit_menu_flag = 1;
    runState = CAR_STABLE;
    while (TRUE)
    {
        // 此处编写需要循环执行的代码
        // printf("%f, %f, %f\n", currentFrontAngle, currentSideAngle, yawAngle);
        // printf("%d\n", runState);
        lcd_show_string(0, 0, "Bottom:");
        lcd_show_int(8, 0, get_bottom_duty(), 3);
        lcd_show_string(0, 1, "Side:");
        lcd_show_int(8, 1, get_side_duty(), 3);

        lcd_show_float(0, 2, (currentFrontAngle - g_euler_angle_bias.pitch), 3,
                       3);
        lcd_show_float(8, 2,
                       currentFrontAngle - g_euler_angle_bias.pitch -
                           g_control_target.sideAngle,
                       3, 3);
        lcd_show_float(0, 3, (currentSideAngle - g_euler_angle_bias.roll), 3,
                       3);
        lcd_show_float(8, 3,
                       currentSideAngle - g_euler_angle_bias.roll -
                           g_control_target.frontAngle,
                       3, 3);
        lcd_show_int(0, 4, runState, 3);

        // lcd_show_string(0, 5, "Pitch:");
        // lcd_show_float(8, 5, currentFrontAngle, 3, 3);
        // lcd_show_string(0, 6, "Row:");
        // lcd_show_float(8, 6, currentSideAngle, 3, 3);
        // lcd_show_string(0, 7, "Yaw:");
        // lcd_show_float(8, 7, yawAngle, 3, 3);

        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore

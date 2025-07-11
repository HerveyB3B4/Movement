/*********************************************************************************************************************
 * NXP RT1021DAG5A Opensourec Library 即（NXP RT1021DAG5A 开源库）是一个基于官方
 *SDK 接口的第三方开源库 Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 NXP RT1021DAG5A 开源库的一部分
 *
 * NXP RT1021DAG5A 开源库 是免费软件
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
 * 文件名称          zf_device_imu660rb
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.33
 * 适用平台          NXP RT1021DAG5A
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-21        SeekFree            first version
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * 接线定义：
 *                   ------------------------------------
 *                   模块管脚            单片机管脚
 *                   // 硬件 SPI 引脚
 *                   SCL/SPC           查看 zf_device_imu660rb.h 中
 *IMU660RB_SPC_PIN 宏定义 SDA/DSI           查看 zf_device_imu660rb.h 中
 *IMU660RB_SDI_PIN 宏定义 SA0/SDO           查看 zf_device_imu660rb.h 中
 *IMU660RB_SDO_PIN 宏定义 CS                查看 zf_device_imu660rb.h 中
 *IMU660RB_CS_PIN 宏定义 VCC               3.3V电源 GND               电源地
 *                   其余引脚悬空
 *
 *                   // 软件 IIC 引脚
 *                   SCL/SPC           查看 zf_device_imu660rb.h 中
 *IMU660RB_SCL_PIN 宏定义 SDA/DSI           查看 zf_device_imu660rb.h 中
 *IMU660RB_SDA_PIN 宏定义 VCC               3.3V电源 GND               电源地
 *                   其余引脚悬空
 *                   ------------------------------------
 ********************************************************************************************************************/

#ifndef _zf_device_imu660rb_h_
#define _zf_device_imu660rb_h_

#include "zf_common_typedef.h"

// IMU660RB_USE_SOFT_IIC定义为0表示使用硬件SPI驱动 定义为1表示使用软件IIC驱动
// 当更改IMU660RB_USE_SOFT_IIC定义后，需要先编译并下载程序，单片机与模块需要断电重启才能正常通讯
#define IMU660RB_USE_SOFT_IIC (0) // 默认使用硬件 SPI 方式驱动

#if IMU660RB_USE_SOFT_IIC // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 IIC
// 驱动====================================================
#define IMU660RB_SOFT_IIC_DELAY \
    (59) // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
#define IMU660RB_SCL_PIN \
    (P20_11) // 软件 IIC SCL 引脚 连接 IMU660RB 的 SCL 引脚
#define IMU660RB_SDA_PIN \
    (P20_14) // 软件 IIC SDA 引脚 连接 IMU660RB 的 SDA 引脚
//====================================================软件 IIC
// 驱动====================================================
#else

//====================================================硬件 SPI
// 驱动====================================================
#define IMU660RB_SPI_SPEED (10 * 1000 * 1000) // 硬件 SPI 速率
#define IMU660RB_SPI IMU_SPI                  // 硬件 SPI 号
#define IMU660RB_SPC_PIN IMU_SPC_PIN          // 硬件 SPI SCK 引脚
#define IMU660RB_SDI_PIN IMU_SDI_PIN          // 硬件 SPI MOSI 引脚
#define IMU660RB_SDO_PIN IMU_SDO_PIN          // 硬件 SPI MISO 引脚
//====================================================硬件 SPI
// 驱动====================================================
#endif
#define IMU660RB_CS_PIN IMU_CS_PIN // CS 片选引脚
#define IMU660RB_CS(x) ((x) ? (gpio_high(IMU660RB_CS_PIN)) : (gpio_low(IMU660RB_CS_PIN)))

#define IMU660RB_TIMEOUT_COUNT (0x00FF) // IMU660 超时计数

//================================================定义 imu660rb
// 内部地址================================================
#define IMU660RB_DEV_ADDR (0x6B) // SA0接地：0x68 SA0上拉：0x69 模块默认上拉
#define IMU660RB_SPI_W (0x00)
#define IMU660RB_SPI_R (0x80)

#define IMU660RB_CHIP_ID (0x0F)

#define IMU660RB_INT1_CTRL (0x0D)
#define IMU660RB_CTRL1_XL (0x10)
#define IMU660RB_CTRL2_G (0x11)
#define IMU660RB_CTRL3_C (0x12)
#define IMU660RB_CTRL4_C (0x13)
#define IMU660RB_CTRL5_C (0x14)
#define IMU660RB_CTRL6_C (0x15)
#define IMU660RB_CTRL7_G (0x16)
#define IMU660RB_CTRL9_XL (0x18)

#define IMU660RB_ACC_ADDRESS (0x28)
#define IMU660RB_GYRO_ADDRESS (0x22)

#define IMU660RB_ACC_SAMPLE (0x3C) // 加速度计量程
// 设置为:0x30 加速度量程为:±2G      获取到的加速度计数据
// 除以16393，可以转化为带物理单位的数据，单位：g(m/s^2) 设置为:0x38
// 加速度量程为:±4G      获取到的加速度计数据 除以8197，
// 可以转化为带物理单位的数据，单位：g(m/s^2) 设置为:0x3C 加速度量程为:±8G
// 获取到的加速度计数据 除以4098， 可以转化为带物理单位的数据，单位：g(m/s^2)
// 设置为:0x34 加速度量程为:±16G     获取到的加速度计数据 除以2049，
// 可以转化为带物理单位的数据，单位：g(m/s^2)

// ACC_SAMPLE 第一位表示采样率。修改此处记得修改imu660rb_acc_transition函数中的switch语句
// 0 powerdown
// B 1.6Hz
// 1 12.5Hz
// 2 26Hz
// 3 52Hz
// 4 104Hz
// 5 208Hz
// 6 416Hz
// 7 833Hz
// 8 1667Hz
// 9 3333Hz
// A 6667Hz
// C以上 Not allowed

#define IMU660RB_GYR_SAMPLE (0xA1) // 陀螺仪量程
// 设置为:0x52 陀螺仪量程为:±125dps  获取到的陀螺仪数据除以228.6，
// 可以转化为带物理单位的数据，单位为：°/s 设置为:0x50 陀螺仪量程为:±250dps
// 获取到的陀螺仪数据除以114.3，   可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x54 陀螺仪量程为:±500dps  获取到的陀螺仪数据除以57.1，
// 可以转化为带物理单位的数据，单位为：°/s 设置为:0x58 陀螺仪量程为:±1000dps
// 获取到的陀螺仪数据除以28.6，    可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x5C 陀螺仪量程为:±2000dps 获取到的陀螺仪数据除以14.3，
// 可以转化为带物理单位的数据，单位为：°/s 设置为:0x51 陀螺仪量程为:±4000dps
// 获取到的陀螺仪数据除以7.1，     可以转化为带物理单位的数据，单位为：°/s

// GYR_SAMPLE 第一位表示采样率。修改此处记得修改imu660rb_gyro_transition函数中的switch语句
// 0 powerdown
// 1 12.5Hz
// 2 26Hz
// 3 52Hz
// 4 104Hz
// 5 208Hz
// 6 416Hz
// 7 833Hz
// 8 1667Hz
// 9 3333Hz
// A 6667Hz
// B以上 Not available

//================================================声明 IMU963RB 全局变量================================================
extern int16 imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z; // 三轴陀螺仪数据      gyro (陀螺仪)
extern int16 imu660rb_acc_x, imu660rb_acc_y, imu660rb_acc_z;    // 三轴加速度计数据     acc (accelerometer 加速度计)
//================================================声明 IMU963RB 全局变量================================================

//================================================声明 IMU963RB 基础函数================================================
void imu660rb_get_acc(void);                      // 获取 IMU660RB 加速度计数据
void imu660rb_get_gyro(void);                     // 获取 IMU660RB 陀螺仪数据
float imu660rb_acc_transition(int16 acc_value);   // 将 IMU660RB 加速度计数据转换为实际物理数据
float imu660rb_gyro_transition(int16 gyro_value); // 将 IMU660RB 陀螺仪数据转换为实际物理数据
uint8 imu660rb_init(void);                        // 初始化 IMU660RB
//================================================声明 IMU963RB 基础函数================================================

#endif

/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          zf_device_mt9v03x
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚             单片机管脚
*                  TXD                查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX 宏定义
*                  RXD                查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX 宏定义
*                  PCLK               查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN 宏定义
*                  VSY                查看 zf_device_mt9v03x.h 中 MT9V03X_VSYNC_PIN 宏定义
*                  D0-D7              查看 zf_device_mt9v03x.h 中 MT9V03X_DATA_PIN 宏定义 从该定义开始的连续八个引脚
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*                  ------------------------------------
********************************************************************************************************************/

#include "zf_common_interrupt.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_exti.h"
#include "zf_driver_gpio.h"
#include "zf_device_camera.h"
#include "zf_device_config.h"
#include "isr_config.h"
#include "zf_device_mt9v03x2.h"
#include "zf_device_mt9v03x.h"
#pragma section all "cpu1_dsram"

vuint8  mt9v03x_finish_flag = 0;                            // 一场图像采集完成标志位
IFX_ALIGN(4) uint8  mt9v03x_image[MT9V03X_H][MT9V03X_W];    // 必须4字节对齐

static  m9v03x_type_enum mt9v03x_type;                      // 定义摄像头类型

int16   timeout = MT9V03X_INIT_TIMEOUT;                     // 定义超时溢出时长

uint8   mt9v03x_lost_flag = 1;                              // 图像丢失标志位
uint8   mt9v03x_dma_int_num;                                // 当前DMA中断次数
uint8   mt9v03x_dma_init_flag;                              // 是否需要重新初始化
uint8   mt9v03x_link_list_num;

int16   mt9v03x_dma_run_flag = 0;
uint8   mt9v03x_init_flag = 0;
// 需要配置到摄像头的数据 不允许在这修改参数
static int16 mt9v03x_set_confing_buffer[MT9V03X_CONFIG_FINISH][2]=
{
    {MT9V03X_INIT,              0},                                             // 摄像头开始初始化
    {MT9V03X_AUTO_EXP,          MT9V03X_AUTO_EXP_DEF},                          // 自动曝光设置   范围1-63 0为关闭 如果自动曝光开启  EXP_TIME命令设置的数据将会变为最大曝光时间，也就是自动曝光时间的上限
    {MT9V03X_EXP_TIME,          MT9V03X_EXP_TIME_DEF},                          // 曝光时间      摄像头收到后会自动计算出最大曝光时间，如果设置过大则设置为计算出来的最大曝光值
    {MT9V03X_FPS,               MT9V03X_FPS_DEF},                               // 图像帧率      摄像头收到后会自动计算出最大FPS，如果过大则设置为计算出来的最大FPS
    {MT9V03X_SET_COL,           MT9V03X_W},                                     // 图像列数量    范围1-752
    {MT9V03X_SET_ROW,           MT9V03X_H},                                     // 图像行数量    范围1-480
    {MT9V03X_LR_OFFSET,         MT9V03X_LR_OFFSET_DEF},                         // 图像左右偏移量  正值 右偏移   负值 左偏移  列为188 376 752时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_UD_OFFSET,         MT9V03X_UD_OFFSET_DEF},                         // 图像上下偏移量  正值 上偏移   负值 下偏移  行为120 240 480时无法设置偏移    摄像头收偏移数据后会自动计算最大偏移，如果超出则设置计算出来的最大偏移
    {MT9V03X_GAIN,              MT9V03X_GAIN_DEF},                              // 图像增益      范围16-64     增益可以在曝光时间固定的情况下改变图像亮暗程度
    {MT9V03X_PCLK_MODE,         MT9V03X_PCLK_MODE_DEF},                         // 像素时钟模式   仅总钻风MT9V034 V2.0以及以上版本支持该命令
};


//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头场中断
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_vsync_handler();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_vsync_handler(void)
{
    exti_flag_clear(MT9V03X_VSYNC_PIN);

    if ((mt9v03x2_dma_run_flag == 1) && (mt9v03x2_init_flag))                   //  分时采集
    {
        return ;
    }
    mt9v03x_dma_int_num = 0;
    if(mt9v03x_dma_init_flag )
    {
        mt9v03x_dma_init_flag = 0;
        IfxDma_resetChannel(&MODULE_DMA, MT9V03X_DMA_CH);
        mt9v03x_link_list_num = dma_init(MT9V03X_DMA_CH,
                                         MT9V03X_DATA_ADD,
                                         mt9v03x_image[0],
                                         MT9V03X_PCLK_PIN,
                                         EXTI_TRIGGER_FALLING,
                                         MT9V03X_IMAGE_SIZE,
                                         DMA_INT_PRIO);           // 如果超频到300M 倒数第二个参数请设置为FALLING
        dma_enable(MT9V03X_DMA_CH);
    }
    else
    {
        if(1 == mt9v03x_link_list_num)
        {
            dma_set_destination(MT9V03X_DMA_CH, mt9v03x_image[0]);      // 没有采用链接传输模式 重新设置目的地址
        }
        dma_enable(MT9V03X_DMA_CH);

    }

    mt9v03x_lost_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      MT9V03X摄像头DMA完成中断
//  参数说明      void
//  返回参数      void
//  使用示例      mt9v03x_dma_handler();
//-------------------------------------------------------------------------------------------------------------------
static void mt9v03x_dma_handler(void)
{


    clear_dma_flag(MT9V03X_DMA_CH);

//    if(IfxDma_getChannelTransactionRequestLost(&MODULE_DMA, MT9V03X_DMA_CH)) // 图像错位判断
//    {
//        mt9v03x_finish_flag = 0;
//        dma_disable(MT9V03X_DMA_CH);
//        IfxDma_clearChannelTransactionRequestLost(&MODULE_DMA, MT9V03X_DMA_CH);
//        mt9v03x_dma_init_flag = 1;
//    }
//    else
    {
        mt9v03x_dma_int_num++;
        if(mt9v03x_dma_int_num >= mt9v03x_link_list_num)
        {
            // 采集完成
            // 一副图像从采集开始到采集结束耗时3.8MS左右(50FPS、188*120分辨率)
            mt9v03x_dma_int_num = 0;
            mt9v03x_lost_flag   = 0;
            mt9v03x_finish_flag = 1;


            mt9v03x_dma_run_flag = 0;
            mt9v03x2_dma_run_flag = 1;
            dma_disable(MT9V03X_DMA_CH);

        }
    }
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     MT9V03X 摄像头初始化
// 参数说明     void
// 返回参数     uint8           1-失败 0-成功
// 使用示例     zf_log(mt9v03x_init(), "mt9v03x init error");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 mt9v03x_init (void)
{
    uint8 return_state = 0;
    soft_iic_info_struct mt9v03x_iic_struct;
    do
    {
        system_delay_ms(200);
        // 首先尝试SCCB通讯
        mt9v03x_type = MT9V03X_SCCB;
        set_camera_type(CAMERA_GRAYSCALE, mt9v03x_vsync_handler, mt9v03x_dma_handler, NULL);
        soft_iic_init(&mt9v03x_iic_struct, 0, MT9V03X_COF_IIC_DELAY, MT9V03X_COF_IIC_SCL, MT9V03X_COF_IIC_SDA);
        if(mt9v03x_set_config_sccb(&mt9v03x_iic_struct, mt9v03x_set_confing_buffer))
        {
            // SCCB通讯失败
            zf_log(0, "MT9V03X set sccb error.");
            return_state = 1;
            break;
        }
        mt9v03x_link_list_num = camera_init(MT9V03X_DATA_ADD, mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    }while(0);

    mt9v03x_init_flag = 1;
    return return_state;
}

#pragma section all restore
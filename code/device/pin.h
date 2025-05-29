#ifndef _PIN_H_
#define _PIN_H_

// camera
#define CAMERA1_COF_IIC_SCL (P10_5)
#define CAMERA1_COF_IIC_SDA (P10_6)
#define CAMERA1_VSYNC_PIN (ERU_CH5_REQ1_P15_8) // 场中断引脚
#define CAMERA1_PCLK_PIN (ERU_CH0_REQ0_P15_4)  // PCLK 触发信号 TIM_ETR 引脚禁止随意修改
#define CAMERA1_DATA_PIN (P02_0)
#define CAMERA1_DMA_CH (IfxDma_ChannelId_7)

#define CAMERA2_COF_IIC_SCL (P00_8)
#define CAMERA2_COF_IIC_SDA (P10_1)
#define CAMERA2_VSYNC_PIN (ERU_CH2_REQ2_P10_2) // 场中断引脚
#define CAMERA2_PCLK_PIN (ERU_CH3_REQ3_P10_3)  // PCLK 触发信号 TIM_ETR 引脚禁止随意修改
#define CAMERA2_DATA_PIN (P00_0)               // 数据引脚 这里是 只能是 GPIOx0 或者 GPIOx8 开始 连续八个引脚例如 P00_0-P00_7
#define CAMERA2_DMA_CH (IfxDma_ChannelId_6)

#define IMU_SPI (SPI_2)
#define IMU_SPC_PIN (SPI2_SCLK_P15_3) // 硬件 SPI SCK 引脚
#define IMU_SDI_PIN (SPI2_MOSI_P15_5) // 硬件 SPI MOSI 引脚
#define IMU_SDO_PIN (SPI2_MISO_P15_7) // 硬件 SPI MISO 引脚
#define IMU_CS_PIN (P14_6)            // CS 片选引脚

#define ENCODER_BOTTOM TIM4_ENCODER
#define ENCODER_PIN0_BOTTOM TIM4_ENCODER_CH1_P02_8
#define ENCODER_PIN1_BOTTOM TIM4_ENCODER_CH2_P00_9

#endif
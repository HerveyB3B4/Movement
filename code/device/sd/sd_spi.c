#include "sd_spi.h"

/**
 * @brief SD卡类型标识
 * 0: 无效卡
 * 1: MMC v3卡
 * 2: SD v1卡
 * 4: SD v2卡
 * 6: SD v2高容量卡
 */
uint8 sd_type = 0;

/**
 * @brief SPI读写一个字节
 * @param data 要发送的数据
 * @return 接收到的数据
 */
uint8 sd_spi_readWriteByte(uint8 data)
{
    uint8 modata = data;
    uint8 midata;
    spi_transfer_8bit(SD_SPIN, &modata, &midata, 1);
    return midata;
}

/**
 * @brief 等待SD卡就绪
 * @return 0:准备好 1:超时未就绪
 */
uint8 sd_waitReady(void)
{
    uint32 t = 0;
    do
    {
        if (sd_spi_readWriteByte(0xFF) == 0xFF)
        {
            return 0; // 准备好
        }
        t++; // 增加计数器值，防止死循环
    } while (t < 0xFFFFFF); // 等待超时
    return 1; // 超时错误
}

/**
 * @brief 取消片选，释放SPI总线
 */
void sd_disSelect(void)
{
    SD_CS_DISABLE;
    sd_spi_readWriteByte(0xff); // 提供额外8个时钟
}

/**
 * @brief 选中SD卡，并等待卡准备好
 * @return 0:成功 1:失败
 */
uint8 sd_select(void)
{
    SD_CS_ENABLE;
    if (sd_waitReady() == 0)
    {
        return 0; // 等待成功
    }
    sd_disSelect();
    return 1; // 等待失败
}

/**
 * @brief 向SD卡发送命令
 * @param cmd 命令
 * @param arg 命令参数
 * @param crc CRC校验值
 * @return 响应值
 */
uint8 sd_sendCmd(uint8 cmd, uint32 arg, uint8 crc)
{
    uint8 rl;
    uint8 retry;

    sd_disSelect(); // 取消上次片选
    if (sd_select())
    {
        return 0xFF; // 片选失败
    }

    // 发送命令序列
    sd_spi_readWriteByte((uint8)(cmd | 0x40)); // 命令
    sd_spi_readWriteByte((uint8)(arg >> 24));  // 参数
    sd_spi_readWriteByte((uint8)(arg >> 16));
    sd_spi_readWriteByte((uint8)(arg >> 8));
    sd_spi_readWriteByte((uint8)arg);
    sd_spi_readWriteByte(crc); // CRC

    // CMD12停止传输命令需要额外一个字节
    if (cmd == CMD12)
    {
        sd_spi_readWriteByte(0xFF);
    }

    // 等待响应，最多等待0x1F次
    retry = 0x1F;
    do
    {
        rl = sd_spi_readWriteByte(0xFF);
    } while ((rl & 0x80) && retry--);

    return rl; // 返回响应值
}

/**
 * @brief 向SD卡发送数据块
 * @param buf 数据缓存区
 * @param cmd 命令
 * @return 0:成功 1:等待失败 2:响应错误
 */
uint8 sd_sendBlock(uint8 *buf, uint8 cmd)
{
    uint16 t;

    if (sd_waitReady())
    {
        return 1; // 等待准备失败
    }

    sd_spi_readWriteByte(cmd);

    if (cmd != 0xFD) // 不是结束指令
    {
        // 发送512字节数据
        for (t = 0; t < 512; t++)
        {
            sd_spi_readWriteByte(buf[t]);
        }

        // 发送两个字节的CRC（不进行实际校验）
        sd_spi_readWriteByte(0xFF);
        sd_spi_readWriteByte(0xFF);

        // 获取响应
        t = sd_spi_readWriteByte(0xFF);
        if ((t & 0x1F) != 0x05)
        {
            return 2; // 响应错误
        }
    }
    return 0; // 写入成功
}

/**
 * @brief 设置SPI低速模式
 */
void sd_spi_speedLow(void)
{
    spi_init(SD_SPIN, 3, 28 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
}

/**
 * @brief 设置SPI高速模式
 */
void sd_spi_speedHigh(void)
{
    spi_init(SD_SPIN, 3, 36 * 1000 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
}

/**
 * @brief 初始化SD卡的SPI接口
 */
void sd_spi_init(void)
{
    // 初始化为低速模式
    spi_init(SD_SPIN, 3, 28 * 1000, SD_SCL, SD_SDA, SD_SDA_IN, SD_CS);
    gpio_init(SD_CS_PIN, GPO, 1, GPO_PUSH_PULL);
    SD_CS_DISABLE;
}

/**
 * @brief 等待SD卡的指定响应
 * @param response 期待的响应值
 * @return 0:成功 1:超时
 */
uint8 sd_getResponse(uint8 response)
{
    uint16 count = 0xFFFF;

    while ((sd_spi_readWriteByte(0xFF) != response) && count)
    {
        count--;
    }

    return (count == 0) ? MSD_RESPONSE_FAILURE : MSD_RESPONSE_NO_ERROR;
}

/**
 * @brief 从SD卡读取数据
 * @param buf 数据缓存区
 * @param len 要读取的数据长度
 * @return 0:成功 1:失败
 */
uint8 sd_recvData(uint8 *buf, uint16 len)
{
    // 等待SD卡发送数据起始令牌0xFE
    if (sd_getResponse(0xFE))
    {
        return 1; // 超时错误
    }

    // 接收数据
    while (len--)
    {
        *buf++ = sd_spi_readWriteByte(0xFF);
    }

    // 忽略CRC校验值
    sd_spi_readWriteByte(0xFF);
    sd_spi_readWriteByte(0xFF);

    return 0; // 读取成功
}

/**
 * @brief 获取SD卡的CSD信息
 * @param csd_data 存放CSD的内存（至少16字节）
 * @return 0:成功 1:失败
 */
uint8 sd_getCSD(uint8 *csd_data)
{
    uint8 rl;

    // 发送CMD9命令，读取CSD
    rl = sd_sendCmd(CMD9, 0, 0x01);
    if (rl == 0)
    {
        rl = sd_recvData(csd_data, 16);
    }

    sd_disSelect();
    return rl ? 1 : 0;
}

/**
 * @brief 获取SD卡的扇区数
 * @return 0:获取失败 其他:SD卡扇区数
 */
uint32 sd_getSectorCount(void)
{
    uint8 csd[16];
    uint32 capacity;
    uint8 n;
    uint16 csize;

    // 获取CSD信息
    if (sd_getCSD(csd) != 0)
    {
        return 0;
    }

    // 根据CSD信息计算扇区数
    if ((csd[0] & 0xC0) == 0x40) // SDv2
    {
        csize = csd[9] + ((uint16)csd[8] << 8) + 1;
        capacity = (uint32)csize << 10; // 扇区数 = (C_SIZE+1) * 1024
    }
    else // SDv1或MMC
    {
        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
        csize = (csd[8] >> 6) + ((uint16)csd[7] << 2) + ((uint16)(csd[6] & 3) << 10) + 1;
        capacity = (uint32)csize << (n - 9); // 得到扇区数
    }

    return capacity;
}

/**
 * @brief 初始化SD卡
 * @return 0:成功 其他:错误代码
 */
uint8 sd_initialize(void)
{
    uint8 rl;
    uint16 retry;
    uint8 buf[4];
    uint16 i;

    // 初始化SPI接口
    sd_spi_init();

    // 发送至少74个时钟脉冲
    for (i = 0; i < 10; ++i)
    {
        sd_spi_readWriteByte(0xFF);
    }

    // 尝试进入IDLE状态
    retry = 20;
    do
    {
        rl = sd_sendCmd(CMD0, 0, 0x95);
    } while ((rl != 0x01) && retry--);

    sd_type = 0; // 默认无卡

    if (rl == 0x01) // 进入IDLE状态成功
    {
        // 检查是否是SD v2.0卡
        if (sd_sendCmd(CMD8, 0x1AA, 0x87) == 1)
        {
            // 获取R7响应
            for (i = 0; i < 4; ++i)
            {
                buf[i] = sd_spi_readWriteByte(0xFF);
            }

            // 检查电压范围
            if (buf[2] == 0x01 && buf[3] == 0xAA)
            {
                // 等待退出IDLE状态
                retry = 0xFFFE;
                do
                {
                    sd_sendCmd(CMD55, 0, 0x01);
                    rl = sd_sendCmd(CMD41, 0x40000000, 0x01);
                } while (rl && retry--);

                // 检查OCR信息
                if (retry && sd_sendCmd(CMD58, 0, 0x01) == 0)
                {
                    for (i = 0; i < 4; ++i)
                    {
                        buf[i] = sd_spi_readWriteByte(0xFF);
                    }

                    // 判断是否是高容量卡
                    sd_type = (buf[0] & 0x40) ? SD_TYPE_V2HC : SD_TYPE_V2;
                }
            }
        }
        else // SD v1.x或MMC v3
        {
            // 先尝试SD v1卡
            sd_sendCmd(CMD55, 0, 0x01);
            rl = sd_sendCmd(CMD41, 0, 0x01);

            if (rl <= 1) // SD v1卡
            {
                sd_type = SD_TYPE_V1;

                // 等待退出IDLE状态
                retry = 0xFFFE;
                do
                {
                    sd_sendCmd(CMD55, 0, 0x01);
                    rl = sd_sendCmd(CMD41, 0, 0x01);
                } while (rl && retry--);
            }
            else // MMC卡
            {
                sd_type = SD_TYPE_MMC;

                // 等待退出IDLE状态
                retry = 0xFFFE;
                do
                {
                    rl = sd_sendCmd(CMD1, 0, 0x01);
                } while (rl && retry--);
            }

            // 设置块大小为512字节
            if (retry == 0 || sd_sendCmd(CMD16, 512, 0x01) != 0)
            {
                sd_type = SD_TYPE_ERR;
            }
        }
    }

    sd_disSelect();
    sd_spi_speedHigh(); // 切换到高速模式

    if (sd_type)
    {
        return 0; // 初始化成功
    }
    else if (rl)
    {
        return rl; // 返回错误代码
    }

    return 0xaa; // 其他错误
}

/**
 * @brief 向SD卡写扇区
 * @param buf 数据缓冲区
 * @param sector 扇区地址
 * @param cnt 扇区数
 * @return 0:成功 其他:错误代码
 */
uint8 sd_writeDisk(uint8 *buf, uint32 sector, uint8 cnt)
{
    uint8 rl;

    // 对于非高容量卡，需要将扇区地址转换为字节地址
    if (sd_type != SD_TYPE_V2HC)
    {
        sector *= 512;
    }

    // 单扇区写入
    if (cnt == 1)
    {
        rl = sd_sendCmd(CMD24, sector, 0x01);
        if (rl == 0)
        {
            rl = sd_sendBlock(buf, 0xFE);
        }
    }
    else // 多扇区写入
    {
        // 对于SD卡，预设写入块数
        if (sd_type != SD_TYPE_MMC)
        {
            sd_sendCmd(CMD55, 0, 0x01);
            sd_sendCmd(CMD23, cnt, 0x01);
        }

        // 发送多块写入命令
        rl = sd_sendCmd(CMD25, sector, 0x01);
        if (rl == 0)
        {
            do
            {
                rl = sd_sendBlock(buf, 0xFC);
                buf += 512;
            } while (--cnt && rl == 0);

            // 发送结束传输标记
            rl = sd_sendBlock(0, 0xFD);
        }
    }

    sd_disSelect();
    return rl;
}

/**
 * @brief 从SD卡读取扇区
 * @param buf 数据缓冲区
 * @param sector 扇区地址
 * @param cnt 扇区数
 * @return 0:成功 其他:错误代码
 */
uint8 sd_readDisk(uint8 *buf, uint32 sector, uint8 cnt)
{
    uint8 rl;

    // 对于非高容量卡，需要将扇区地址转换为字节地址
    if (sd_type != SD_TYPE_V2HC)
    {
        sector <<= 9; // sector *= 512
    }

    // 单扇区读取
    if (cnt == 1)
    {
        rl = sd_sendCmd(CMD17, sector, 0x01);
        if (rl == 0)
        {
            rl = sd_recvData(buf, 512);
        }
    }
    else // 多扇区读取
    {
        rl = sd_sendCmd(CMD18, sector, 0x01);
        do
        {
            rl = sd_recvData(buf, 512);
            buf += 512;
        } while (--cnt && rl == 0);

        // 发送停止传输命令
        sd_sendCmd(CMD12, 0, 0x01);
    }

    sd_disSelect();
    return rl;
}

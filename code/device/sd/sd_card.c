#include "sd_card.h"

static uint8 sd_initialized = 0; // SD卡状态标志

// SD卡数据区域信息结构体

// 魔术字常量，用于验证信息结构体

// 信息结构体
static sd_data_info_t sd_info = {0, 0, {0, 0}};

/**
 * @brief 保存SD卡数据信息
 * @return SD_OK:成功 其他:错误代码
 */
static sd_result_t sd_save_info(void)
{
    uint8 buffer[SD_SECTOR_SIZE] = {0};

    // 设置魔术字
    sd_info.magic = SD_INFO_MAGIC;

    // 将结构体拷贝到缓冲区
    memcpy(buffer, &sd_info, sizeof(sd_data_info_t));

    // 将缓冲区写入SD卡
    uint8 res = sd_writeDisk(buffer, SD_INFO_SECTOR, 1);
    return (res == 0) ? SD_OK : SD_ERROR;
}

/**
 * @brief 加载SD卡数据信息
 * @return SD_OK:成功 其他:错误代码
 */
static sd_result_t sd_load_info(void)
{
    uint8 buffer[SD_SECTOR_SIZE] = {0};

    // 读取SD卡信息扇区
    uint8 res = sd_readDisk(buffer, SD_INFO_SECTOR, 1);
    if (res != 0)
    {
        return SD_ERROR;
    }

    // 将缓冲区数据拷贝到结构体
    memcpy(&sd_info, buffer, sizeof(sd_data_info_t));

    // 验证魔术字
    if (sd_info.magic != SD_INFO_MAGIC)
    {
        // 魔术字不匹配，表示未初始化过数据区
        sd_info.magic = 0;
        sd_info.data_size = 0;
        return SD_ERROR;
    }

    return SD_OK;
}

/**
 * @brief 初始化SD卡
 * @return SD_OK:成功 其他:错误代码
 */
sd_result_t sd_init(void)
{
    uint8 retry = 5;
    uint8 res;

    // 尝试多次初始化SD卡
    while (retry--)
    {
        res = sd_initialize();
        if (res == 0)
        {
            break;
        }
    }

    if (res)
    {
        sd_initialized = 0;
        return SD_INIT_FAILED;
    }

    // 设置已初始化标志
    sd_initialized = 1;

    // 尝试加载数据信息
    if (sd_load_info() != SD_OK)
    {
        // 如果加载失败，初始化默认值并保存
        sd_info.magic = SD_INFO_MAGIC;
        sd_info.data_size = 0;
        sd_save_info();
    }

    return SD_OK;
}

/**
 * @brief 向SD卡写入数据
 * @param data 数据缓冲区
 * @param size 数据大小(字节)
 * @param mode 写入模式(覆盖/追加)
 * @return SD_OK:成功 其他:错误代码
 */
sd_result_t sd_write_data(uint8 *data, uint32 size, sd_write_mode_t mode)
{
    uint8 res;
    uint32 start_sector, offset, required_sectors;
    uint32 curr_sector, remain_size, copy_size;
    uint8 buffer[SD_SECTOR_SIZE];

    if (!sd_initialized)
    {
        return SD_NOT_INITIALIZED;
    }

    if (data == NULL || size == 0)
    {
        return SD_ERROR;
    }

    // 根据写入模式确定起始位置
    if (mode == SD_WRITE_OVERRIDE)
    {
        // 覆盖模式：从起始位置开始写入
        sd_info.data_size = 0;
    }
    // 追加模式：从当前数据末尾开始写入

    // 计算起始扇区和扇区内偏移
    start_sector = SD_DATA_START_SECTOR + (sd_info.data_size / SD_SECTOR_SIZE);
    offset = sd_info.data_size % SD_SECTOR_SIZE;

    // 计算需要多少个扇区
    required_sectors = (offset + size + SD_SECTOR_SIZE - 1) / SD_SECTOR_SIZE;

    // 开始数据写入
    curr_sector = start_sector;
    remain_size = size;

    // 处理第一个扇区（可能需要读-修改-写）
    if (offset > 0)
    {
        // 读取当前扇区
        res = sd_readDisk(buffer, curr_sector, 1);
        if (res != 0)
        {
            return SD_ERROR;
        }

        // 计算本扇区可写入的数据量
        copy_size = (remain_size < (SD_SECTOR_SIZE - offset)) ? remain_size : (SD_SECTOR_SIZE - offset);

        // 复制数据到缓冲区
        memcpy(buffer + offset, data, copy_size);

        // 写回扇区
        res = sd_writeDisk(buffer, curr_sector, 1);
        if (res != 0)
        {
            return SD_ERROR;
        }

        // 更新计数器
        data += copy_size;
        remain_size -= copy_size;
        curr_sector++;
    }

    // 处理完整扇区
    while (remain_size >= SD_SECTOR_SIZE)
    {
        // 直接写入完整扇区
        res = sd_writeDisk((uint8 *)data, curr_sector, 1);
        if (res != 0)
        {
            return SD_ERROR;
        }

        // 更新计数器
        data += SD_SECTOR_SIZE;
        remain_size -= SD_SECTOR_SIZE;
        curr_sector++;
    }

    // 处理最后一个不完整扇区
    if (remain_size > 0)
    {
        memset(buffer, 0, SD_SECTOR_SIZE);
        memcpy(buffer, data, remain_size);

        res = sd_writeDisk(buffer, curr_sector, 1);
        if (res != 0)
        {
            return SD_ERROR;
        }
    }

    // 更新数据大小并保存信息
    sd_info.data_size = (mode == SD_WRITE_OVERRIDE) ? size : (sd_info.data_size + size);
    sd_save_info();

    return SD_OK;
}

/**
 * @brief 从SD卡读取数据
 * @param data 数据缓冲区
 * @param size 要读取的最大大小(字节)
 * @param read_size 实际读取的数据大小
 * @return SD_OK:成功 其他:错误代码
 */
sd_result_t sd_read_data(uint8 *data, uint32 size, uint32 *read_size)
{
    uint8 res;
    uint32 start_sector, sectors_to_read;
    uint32 actual_size;

    if (!sd_initialized)
    {
        return SD_NOT_INITIALIZED;
    }

    if (data == NULL || size == 0)
    {
        return SD_ERROR;
    }

    // 确定实际读取大小
    actual_size = (size > sd_info.data_size) ? sd_info.data_size : size;
    if (read_size != NULL)
    {
        *read_size = actual_size;
    }

    // 没有数据可读
    if (actual_size == 0)
    {
        return SD_OK;
    }

    // 计算起始扇区和需要读取的扇区数
    start_sector = SD_DATA_START_SECTOR;
    sectors_to_read = (actual_size + SD_SECTOR_SIZE - 1) / SD_SECTOR_SIZE;

    // 直接读取所需的扇区
    for (uint32 i = 0; i < sectors_to_read; i++)
    {
        uint8 buffer[SD_SECTOR_SIZE];
        uint32 copy_size;

        // 读取一个扇区
        res = sd_readDisk(buffer, start_sector + i, 1);
        if (res != 0)
        {
            return SD_ERROR;
        }

        // 计算本次需要复制的数据量
        if (actual_size > SD_SECTOR_SIZE)
        {
            copy_size = SD_SECTOR_SIZE;
            actual_size -= SD_SECTOR_SIZE;
        }
        else
        {
            copy_size = actual_size;
            actual_size = 0;
        }

        // 复制数据到目标缓冲区
        memcpy(data, buffer, copy_size);
        data += copy_size;

        if (actual_size == 0)
        {
            break;
        }
    }

    return SD_OK;
}

/**
 * @brief 清空SD卡存储的数据
 * @return SD_OK:成功 其他:错误代码
 */
sd_result_t sd_clear_data(void)
{
    if (!sd_initialized)
    {
        return SD_NOT_INITIALIZED;
    }

    // 只需将数据大小设为0即可逻辑清空
    sd_info.data_size = 0;

    // 保存信息
    return sd_save_info();
}

/**
 * @brief 获取SD卡中已存储的数据大小
 * @return 已存储的数据大小(字节)
 */
uint32 sd_get_data_size(void)
{
    return sd_info.data_size;
}

/**
 * @brief 检查SD卡是否已初始化
 * @return 1:已初始化 0:未初始化
 */
uint8 sd_is_initialized(void)
{
    return sd_initialized;
}

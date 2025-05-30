#ifndef _SD_OPERATIONS_H_
#define _SD_OPERATIONS_H_

#include "zf_common_headfile.h"

#define SD_DATA_START_SECTOR 10  // 数据存储的起始扇区
#define SD_INFO_SECTOR 8         // 数据区域信息结构体在SD卡上的扇区
#define SD_SECTOR_SIZE 512       // 扇区大小（字节）
#define SD_INFO_MAGIC 0x5344434D // "SDCM"

// 文件写入模式
typedef enum
{
    SD_WRITE_OVERRIDE = 0, // 覆盖模式 (清空原有数据再写入)
    SD_WRITE_APPEND        // 追加模式 (在原有数据后追加)
} sd_write_mode_t;

typedef enum
{
    SD_OK = 0,         // 操作成功
    SD_ERROR,          // 一般错误
    SD_INIT_FAILED,    // 初始化失败
    SD_NOT_INITIALIZED // 未初始化
} sd_result_t;

typedef struct
{
    uint32 magic;       // 魔术字，用于验证结构体有效性
    uint32 data_size;   // 已存储的数据大小
    uint32 reserved[2]; // 保留用于将来扩展
} sd_data_info_t;

sd_result_t sd_init(void);
sd_result_t sd_write_data(uint8 *data, uint32 size, sd_write_mode_t mode);
sd_result_t sd_read_data(uint8 *data, uint32 size, uint32 *read_size);
sd_result_t sd_clear_data(void);
uint32 sd_get_data_size(void);
uint8 sd_is_initialized(void);

#endif /* _SD_OPERATIONS_H_ */
#ifndef _IMAGE_H
#define _IMAGE_H
#include "zf_common_headfile.h"

#define IMG_WIDTH MT9V03X_W   // 图像宽度（参考网页1逐飞库配置）
#define IMG_HEIGHT MT9V03X_H  // 图像高度
#define BLOCK_SIZE 32         // 局部阈值块尺寸
#define HIST_BINS 64          // 直方图分箱数（降低内存）
#define MAX_REGIONS 256       // 最大区域数量
#define STACK_SIZE 1024       // 栈大小

typedef struct {
    uint8_t sobel_thresh;  // Sobel梯度阈值
    float k_global;        // 全局系数 [0.5-1.5]
    float k_local;         // 局部系数 [0.3-0.7]
} EdgeConfig;

// 区域信息结构体
typedef struct {
    int16_t min_x;  // 区域最小x坐标
    int16_t min_y;  // 区域最小y坐标
    int16_t max_x;  // 区域最大x坐标
    int16_t max_y;  // 区域最大y坐标
    uint32_t area;  // 区域面积（像素数）
} Region;

// 初始化配置
void edge_config_init(EdgeConfig* config);

// 动态阈值边缘检测（输入灰度图，输出二值图）
void edge_detect_dynamic(uint8_t* input, uint8_t* output, EdgeConfig* config);

// 大津法二值化（输入灰度图，输出二值图）
void binary_otsu(uint8_t* input, uint8_t* output);

// 获取第一个白色区块的中心点
// 参数：二值化图像
// 返回：中心点坐标，如果未找到则x和y均为-1
Point find_white_block_center(uint8_t* binary);

// 查找最大白色连通区域的中心点
Point find_largest_white_region_center(uint8_t* binary);

// 在图像上画十字
// 参数：
// - img: 图像数据
// - center: 十字中心点
// - size: 十字大小(臂长)
// - color: 十字颜色(0x00为黑色，0xFF为白色)
void draw_cross(uint8_t* img, Point center, uint8_t size, uint8_t color);

#endif
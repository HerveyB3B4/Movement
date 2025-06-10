#include "binary.h"
#include "image.h"

static uint32 get_block_sum(const uint32 *integral,
                            uint16 x1,
                            uint16 y1,
                            uint16 x2,
                            uint16 y2,
                            uint16 width);
static void calc_integral_image(const uint8 *input,
                                uint32 *integral,
                                uint16 width,
                                uint16 height);
// 自适应阈值二值化（带积分图优化）
void binary_adaptive(uint8 *input,
                     uint8 *output,
                     uint8 block_size,
                     int8 c)
{
    static uint32 integral[IMG_WIDTH * IMG_HEIGHT];

    // 确保block_size是奇数
    if (!(block_size & 1))
        block_size++;
    uint8 half_block = block_size >> 1;

    // 计算积分图
    calc_integral_image(input, integral, IMG_WIDTH, IMG_HEIGHT);

    // 对每个像素计算局部阈值
    for (uint16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (uint16 x = 0; x < IMG_WIDTH; x++)
        {
            // 计算局部窗口的范围（处理边界情况）
            uint16 x1 = (x > half_block) ? (x - half_block) : 0;
            uint16 y1 = (y > half_block) ? (y - half_block) : 0;
            uint16 x2 = (x + half_block < IMG_WIDTH) ? (x + half_block)
                                                     : (IMG_WIDTH - 1);
            uint16 y2 = (y + half_block < IMG_HEIGHT) ? (y + half_block)
                                                      : (IMG_HEIGHT - 1);

            // 计算窗口内的平均值
            uint32 area = (x2 - x1 + 1) * (y2 - y1 + 1);
            uint32 block_sum =
                get_block_sum(integral, x1, y1, x2, y2, IMG_WIDTH);
            uint8 mean = block_sum / area;

            // 二值化（减去一个常数c以调整阈值）
            uint32 idx = y * IMG_WIDTH + x;
            output[idx] = (input[idx] > mean - c) ? RGB565_WHITE : RGB565_BLACK;
        }
    }
}

// 改进的Otsu二值化（添加时域滤波）
void binary_otsu_improved(uint8 *input, uint8 *output)
{
    static uint8 last_threshold = 128; // 保存上一帧的阈值
    static const float alpha = 0.7f;   // 时域滤波系数 (0.0-1.0)

    uint32 histogram[256] = {0};
    uint32 total_pixels = IMG_WIDTH * IMG_HEIGHT;

    // 1. 计算直方图
    for (uint32 i = 0; i < total_pixels; i++)
    {
        histogram[input[i]]++;
    }

    // 2. 计算最佳阈值
    uint32 sum = 0;
    for (uint16 i = 0; i < 256; i++)
    {
        sum += i * histogram[i];
    }

    uint32 w0 = 0;
    uint32 sum0 = 0;
    uint8 threshold = 0;
    float max_variance = 0;

    for (uint16 t = 0; t < 256; t++)
    {
        w0 += histogram[t];
        if (w0 == 0)
            continue;

        uint32 w1 = total_pixels - w0;
        if (w1 == 0)
            break;

        sum0 += t * histogram[t];
        float u0 = sum0 / (float)w0;
        float u1 = (sum - sum0) / (float)w1;
        float variance = w0 * w1 * (u0 - u1) * (u0 - u1);

        if (variance > max_variance)
        {
            max_variance = variance;
            threshold = t;
        }
    }

    // 应用时域滤波
    threshold = (uint8)(alpha * last_threshold + (1.0f - alpha) * threshold);
    last_threshold = threshold;

    // 3. 二值化
    for (uint32 i = 0; i < total_pixels; i++)
    {
        output[i] = (input[i] > threshold) ? RGB565_WHITE : RGB565_BLACK;
    }
}

// 大津法二值化
void binary_otsu(uint8 *input, uint8 *output)
{
    uint32 histogram[256] = {0}; // 灰度直方图
    uint32 total_pixels = IMG_WIDTH * IMG_HEIGHT;

    // 1. 计算直方图
    for (uint32 i = 0; i < total_pixels; i++)
    {
        histogram[input[i]]++;
    }

    // 2. 计算最佳阈值
    uint32 sum = 0;
    for (uint16 i = 0; i < 256; i++)
    {
        sum += i * histogram[i]; // 计算灰度总和
    }

    uint32 w0 = 0;          // 前景像素数
    uint32 sum0 = 0;        // 前景灰度和
    uint8 threshold = 0;    // 最佳阈值
    float max_variance = 0; // 最大类间方差

    for (uint16 t = 0; t < 256; t++)
    {
        w0 += histogram[t]; // 前景像素累加
        if (w0 == 0)
            continue;

        uint32 w1 = total_pixels - w0; // 背景像素数
        if (w1 == 0)
            break;

        sum0 += t * histogram[t];

        // 计算均值
        float u0 = sum0 / (float)w0;         // 前景均值
        float u1 = (sum - sum0) / (float)w1; // 背景均值

        // 计算类间方差
        float variance = w0 * w1 * (u0 - u1) * (u0 - u1);

        // 更新最大方差
        if (variance > max_variance)
        {
            max_variance = variance;
            threshold = t;
        }
    }

    // 3. 二值化
    for (uint32 i = 0; i < total_pixels; i++)
    {
        output[i] = (input[i] > threshold) ? RGB565_WHITE : RGB565_BLACK;
    }
}

// 计算积分图
static void calc_integral_image(const uint8 *input,
                                uint32 *integral,
                                uint16 width,
                                uint16 height)
{
    // 第一个像素
    integral[0] = input[0];

    // 第一行
    for (uint16 x = 1; x < width; x++)
    {
        integral[x] = integral[x - 1] + input[x];
    }

    // 第一列
    for (uint16 y = 1; y < height; y++)
    {
        integral[y * width] = integral[(y - 1) * width] + input[y * width];
    }

    // 其余像素
    for (uint16 y = 1; y < height; y++)
    {
        for (uint16 x = 1; x < width; x++)
        {
            integral[y * width + x] = input[y * width + x] +
                                      integral[y * width + x - 1] +
                                      integral[(y - 1) * width + x] -
                                      integral[(y - 1) * width + x - 1];
        }
    }
}

static uint32 get_block_sum(const uint32 *integral,
                            uint16 x1,
                            uint16 y1,
                            uint16 x2,
                            uint16 y2,
                            uint16 width)
{
    uint32 sum = integral[y2 * width + x2];
    if (x1 > 0)
        sum -= integral[y2 * width + (x1 - 1)];
    if (y1 > 0)
        sum -= integral[(y1 - 1) * width + x2];
    if (x1 > 0 && y1 > 0)
        sum += integral[(y1 - 1) * width + (x1 - 1)];
    return sum;
}
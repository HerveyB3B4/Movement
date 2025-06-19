#include "binary.h"
#include "image.h"

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

// 二维Otsu二值化
void binary_otsu_2d(uint8 *input, uint8 *output)
{
    // 定义邻域窗口大小
    const uint8 window_size = 3;
    const uint8 half_window = window_size / 2;

    // 二维直方图 [灰度值][邻域均值]
    uint32 hist_2d[256][256];
    uint32 total_pixels = IMG_WIDTH * IMG_HEIGHT;

    // 清空直方图
    memset(hist_2d, 0, sizeof(hist_2d));

    // 计算每个像素的邻域均值并构建二维直方图
    for (uint16 y = half_window; y < IMG_HEIGHT - half_window; y++)
    {
        for (uint16 x = half_window; x < IMG_WIDTH - half_window; x++)
        {
            uint8 pixel_val = input[y * IMG_WIDTH + x];

            // 计算邻域均值
            uint32 sum = 0;
            uint16 count = 0;
            for (int8 dy = -half_window; dy <= half_window; dy++)
            {
                for (int8 dx = -half_window; dx <= half_window; dx++)
                {
                    sum += input[(y + dy) * IMG_WIDTH + (x + dx)];
                    count++;
                }
            }
            uint8 neighbor_mean = sum / count;

            hist_2d[pixel_val][neighbor_mean]++;
        }
    }

    // 寻找最优阈值对(s, t)
    uint8 best_s = 0, best_t = 0;
    float max_variance = 0;

    // 计算总的灰度和及像素数
    uint32 total_sum_i = 0, total_sum_j = 0;
    for (uint16 i = 0; i < 256; i++)
    {
        for (uint16 j = 0; j < 256; j++)
        {
            total_sum_i += i * hist_2d[i][j];
            total_sum_j += j * hist_2d[i][j];
        }
    }

    // 遍历所有可能的阈值对
    for (uint16 s = 1; s < 255; s++)
    {
        for (uint16 t = 1; t < 255; t++)
        {
            uint32 w0 = 0, w1 = 0, w2 = 0, w3 = 0;
            uint32 sum0_i = 0, sum0_j = 0;
            uint32 sum1_i = 0, sum1_j = 0;
            uint32 sum2_i = 0, sum2_j = 0;
            uint32 sum3_i = 0, sum3_j = 0;

            // 将二维直方图分为4个区域
            for (uint16 i = 0; i < 256; i++)
            {
                for (uint16 j = 0; j < 256; j++)
                {
                    if (i <= s && j <= t)
                    {
                        // 区域0: 背景
                        w0 += hist_2d[i][j];
                        sum0_i += i * hist_2d[i][j];
                        sum0_j += j * hist_2d[i][j];
                    }
                    else if (i <= s && j > t)
                    {
                        // 区域1: 边缘
                        w1 += hist_2d[i][j];
                        sum1_i += i * hist_2d[i][j];
                        sum1_j += j * hist_2d[i][j];
                    }
                    else if (i > s && j <= t)
                    {
                        // 区域2: 噪声
                        w2 += hist_2d[i][j];
                        sum2_i += i * hist_2d[i][j];
                        sum2_j += j * hist_2d[i][j];
                    }
                    else
                    {
                        // 区域3: 目标
                        w3 += hist_2d[i][j];
                        sum3_i += i * hist_2d[i][j];
                        sum3_j += j * hist_2d[i][j];
                    }
                }
            }

            // 避免除零
            if (w0 == 0 || w1 == 0 || w2 == 0 || w3 == 0)
                continue;

            // 计算各区域均值
            float u0_i = (float)sum0_i / w0, u0_j = (float)sum0_j / w0;
            float u1_i = (float)sum1_i / w1, u1_j = (float)sum1_j / w1;
            float u2_i = (float)sum2_i / w2, u2_j = (float)sum2_j / w2;
            float u3_i = (float)sum3_i / w3, u3_j = (float)sum3_j / w3;

            // 计算总体均值
            float ut_i = (float)total_sum_i / total_pixels;
            float ut_j = (float)total_sum_j / total_pixels;

            // 计算类间方差
            float variance = w0 * ((u0_i - ut_i) * (u0_i - ut_i) + (u0_j - ut_j) * (u0_j - ut_j)) +
                             w1 * ((u1_i - ut_i) * (u1_i - ut_i) + (u1_j - ut_j) * (u1_j - ut_j)) +
                             w2 * ((u2_i - ut_i) * (u2_i - ut_i) + (u2_j - ut_j) * (u2_j - ut_j)) +
                             w3 * ((u3_i - ut_i) * (u3_i - ut_i) + (u3_j - ut_j) * (u3_j - ut_j));

            if (variance > max_variance)
            {
                max_variance = variance;
                best_s = s;
                best_t = t;
            }
        }
    }

    // 使用找到的最优阈值进行二值化
    for (uint16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (uint16 x = 0; x < IMG_WIDTH; x++)
        {
            uint8 pixel_val = input[y * IMG_WIDTH + x];

            // 对边界像素使用简单处理
            if (y < half_window || y >= IMG_HEIGHT - half_window ||
                x < half_window || x >= IMG_WIDTH - half_window)
            {
                output[y * IMG_WIDTH + x] = (pixel_val > best_s) ? RGB565_WHITE : RGB565_BLACK;
                continue;
            }

            // 计算邻域均值
            uint32 sum = 0;
            uint16 count = 0;
            for (int8 dy = -half_window; dy <= half_window; dy++)
            {
                for (int8 dx = -half_window; dx <= half_window; dx++)
                {
                    sum += input[(y + dy) * IMG_WIDTH + (x + dx)];
                    count++;
                }
            }
            uint8 neighbor_mean = sum / count;

            // 根据二维阈值进行分类
            if (pixel_val > best_s && neighbor_mean > best_t)
            {
                output[y * IMG_WIDTH + x] = RGB565_WHITE; // 目标区域
            }
            else
            {
                output[y * IMG_WIDTH + x] = RGB565_BLACK; // 其他区域
            }
        }
    }
}

// 固定阈值二值化
void binary_threshold(uint8 *input, uint8 *output, uint8 threshold)
{
    uint32 total_pixels = IMG_WIDTH * IMG_HEIGHT;

    // 直接应用固定阈值进行二值化
    for (uint32 i = 0; i < total_pixels; i++)
    {
        output[i] = (input[i] > threshold) ? RGB565_WHITE : RGB565_BLACK;
    }
}
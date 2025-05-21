#include "image.h"

// Sobel算子（
static const int8_t GX[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
static const int8_t GY[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

// 分块统计结构（减少内存占用）
typedef struct {
    uint16_t sum_grad;
    uint16_t max_grad;
} BlockStat;

void edge_config_init(EdgeConfig* config) {
    config->sobel_thresh = 30;
    config->k_global = 0.8f;
    config->k_local = 0.5f;
}

// 优化Sobel计算
static inline int16_t sobel_conv(const uint8_t* img,
                                 const int8_t* kernel,
                                 uint16_t stride) {
    return img[-stride - 1] * kernel[0] + img[-stride] * kernel[1] +
           img[-stride + 1] * kernel[2] + img[-1] * kernel[3] +
           img[0] * kernel[4] + img[1] * kernel[5] +
           img[stride - 1] * kernel[6] + img[stride] * kernel[7] +
           img[stride + 1] * kernel[8];
}

// 分块梯度统计
static void block_grad_stats(uint8_t* input, BlockStat* stats) {
    for (uint16_t by = 0; by < IMG_HEIGHT / BLOCK_SIZE; by++) {
        for (uint16_t bx = 0; bx < IMG_WIDTH / BLOCK_SIZE; bx++) {
            uint16_t max_g = 0, sum_g = 0;

            // 处理每个区块
            for (uint8_t dy = 0; dy < BLOCK_SIZE; dy++) {
                for (uint8_t dx = 0; dx < BLOCK_SIZE; dx++) {
                    uint16_t y = by * BLOCK_SIZE + dy;
                    uint16_t x = bx * BLOCK_SIZE + dx;
                    if (y >= 1 && y < IMG_HEIGHT - 1 && x >= 1 &&
                        x < IMG_WIDTH - 1) {
                        const uint32_t idx = y * IMG_WIDTH + x;
                        int16_t gx = sobel_conv(&input[idx], GX, IMG_WIDTH);
                        int16_t gy = sobel_conv(&input[idx], GY, IMG_WIDTH);
                        uint16_t grad = (abs(gx) + abs(gy)) >> 1;

                        sum_g += grad;
                        if (grad > max_g)
                            max_g = grad;
                    }
                }
            }
            stats[by * (IMG_WIDTH / BLOCK_SIZE) + bx].sum_grad = sum_g;
            stats[by * (IMG_WIDTH / BLOCK_SIZE) + bx].max_grad = max_g;
        }
    }
}

// 动态阈值计算（结合全局与局部）
static uint8_t calc_dynamic_thresh(BlockStat* stats, EdgeConfig* cfg) {
    uint32_t global_sum = 0, global_max = 0;
    uint16_t valid_blocks = 0;

    // 第一遍扫描获取全局特征
    for (uint16_t i = 0;
         i < (IMG_WIDTH / BLOCK_SIZE) * (IMG_HEIGHT / BLOCK_SIZE); i++) {
        if (stats[i].max_grad > cfg->sobel_thresh) {
            global_sum += stats[i].sum_grad;
            if (stats[i].max_grad > global_max)
                global_max = stats[i].max_grad;
            valid_blocks++;
        }
    }

    // 计算动态阈值
    float global_avg =
        (valid_blocks > 0)
            ? (global_sum / (float)(valid_blocks * BLOCK_SIZE * BLOCK_SIZE))
            : 0;

    uint8_t thresh =
        (uint8_t)(global_avg * cfg->k_global + global_max * cfg->k_local);
    return (thresh < cfg->sobel_thresh) ? cfg->sobel_thresh : thresh;
}

void edge_detect_dynamic(uint8_t* input, uint8_t* output, EdgeConfig* config) {
    static BlockStat
        stats[(IMG_WIDTH / BLOCK_SIZE) * (IMG_HEIGHT / BLOCK_SIZE)];

    // 阶段1：分块梯度统计
    block_grad_stats(input, stats);

    // 阶段2：动态阈值计算
    uint8_t dyn_thresh = calc_dynamic_thresh(stats, config);

    // 阶段3：边缘检测（带边界保护）
    for (uint16_t y = 1; y < IMG_HEIGHT - 1; y++) {
        for (uint16_t x = 1; x < IMG_WIDTH - 1; x++) {
            uint32_t idx = y * IMG_WIDTH + x;
            int16_t gx = sobel_conv(&input[idx], GX, IMG_WIDTH);
            int16_t gy = sobel_conv(&input[idx], GY, IMG_WIDTH);
            uint16_t grad = (abs(gx) + abs(gy)) >> 1;
            output[idx] = (grad > dyn_thresh) ? 0xFF : 0x00;
        }
    }
}

// 计算积分图
static void calc_integral_image(const uint8_t* input,
                                uint32_t* integral,
                                uint16_t width,
                                uint16_t height) {
    // 第一个像素
    integral[0] = input[0];

    // 第一行
    for (uint16_t x = 1; x < width; x++) {
        integral[x] = integral[x - 1] + input[x];
    }

    // 第一列
    for (uint16_t y = 1; y < height; y++) {
        integral[y * width] = integral[(y - 1) * width] + input[y * width];
    }

    // 其余像素
    for (uint16_t y = 1; y < height; y++) {
        for (uint16_t x = 1; x < width; x++) {
            integral[y * width + x] = input[y * width + x] +
                                      integral[y * width + x - 1] +
                                      integral[(y - 1) * width + x] -
                                      integral[(y - 1) * width + x - 1];
        }
    }
}

// 使用积分图获取区域和
static uint32_t get_block_sum(const uint32_t* integral,
                              uint16_t x1,
                              uint16_t y1,
                              uint16_t x2,
                              uint16_t y2,
                              uint16_t width) {
    uint32_t sum = integral[y2 * width + x2];
    if (x1 > 0)
        sum -= integral[y2 * width + (x1 - 1)];
    if (y1 > 0)
        sum -= integral[(y1 - 1) * width + x2];
    if (x1 > 0 && y1 > 0)
        sum += integral[(y1 - 1) * width + (x1 - 1)];
    return sum;
}

// 自适应阈值二值化（带积分图优化）
void binary_adaptive(uint8_t* input,
                     uint8_t* output,
                     uint8_t block_size,
                     int8_t c) {
    static uint32_t integral[IMG_WIDTH * IMG_HEIGHT];

    // 确保block_size是奇数
    if (!(block_size & 1))
        block_size++;
    uint8_t half_block = block_size >> 1;

    // 计算积分图
    calc_integral_image(input, integral, IMG_WIDTH, IMG_HEIGHT);

    // 对每个像素计算局部阈值
    for (uint16_t y = 0; y < IMG_HEIGHT; y++) {
        for (uint16_t x = 0; x < IMG_WIDTH; x++) {
            // 计算局部窗口的范围（处理边界情况）
            uint16_t x1 = (x > half_block) ? (x - half_block) : 0;
            uint16_t y1 = (y > half_block) ? (y - half_block) : 0;
            uint16_t x2 = (x + half_block < IMG_WIDTH) ? (x + half_block)
                                                       : (IMG_WIDTH - 1);
            uint16_t y2 = (y + half_block < IMG_HEIGHT) ? (y + half_block)
                                                        : (IMG_HEIGHT - 1);

            // 计算窗口内的平均值
            uint32_t area = (x2 - x1 + 1) * (y2 - y1 + 1);
            uint32_t block_sum =
                get_block_sum(integral, x1, y1, x2, y2, IMG_WIDTH);
            uint8_t mean = block_sum / area;

            // 二值化（减去一个常数c以调整阈值）
            uint32_t idx = y * IMG_WIDTH + x;
            output[idx] = (input[idx] > mean - c) ? RGB565_WHITE : RGB565_BLACK;
        }
    }
}

// 改进的Otsu二值化（添加时域滤波）
void binary_otsu_improved(uint8_t* input, uint8_t* output) {
    static uint8_t last_threshold = 128;  // 保存上一帧的阈值
    static const float alpha = 0.7f;      // 时域滤波系数 (0.0-1.0)

    uint32_t histogram[256] = {0};
    uint32_t total_pixels = IMG_WIDTH * IMG_HEIGHT;

    // 1. 计算直方图
    for (uint32_t i = 0; i < total_pixels; i++) {
        histogram[input[i]]++;
    }

    // 2. 计算最佳阈值
    uint32_t sum = 0;
    for (uint16_t i = 0; i < 256; i++) {
        sum += i * histogram[i];
    }

    uint32_t w0 = 0;
    uint32_t sum0 = 0;
    uint8_t threshold = 0;
    float max_variance = 0;

    for (uint16_t t = 0; t < 256; t++) {
        w0 += histogram[t];
        if (w0 == 0)
            continue;

        uint32_t w1 = total_pixels - w0;
        if (w1 == 0)
            break;

        sum0 += t * histogram[t];
        float u0 = sum0 / (float)w0;
        float u1 = (sum - sum0) / (float)w1;
        float variance = w0 * w1 * (u0 - u1) * (u0 - u1);

        if (variance > max_variance) {
            max_variance = variance;
            threshold = t;
        }
    }

    // 应用时域滤波
    threshold = (uint8_t)(alpha * last_threshold + (1.0f - alpha) * threshold);
    last_threshold = threshold;

    // 3. 二值化
    for (uint32_t i = 0; i < total_pixels; i++) {
        output[i] = (input[i] > threshold) ? RGB565_WHITE : RGB565_BLACK;
    }
}

// 大津法二值化
void binary_otsu(uint8_t* input, uint8_t* output) {
    uint32_t histogram[256] = {0};  // 灰度直方图
    uint32_t total_pixels = IMG_WIDTH * IMG_HEIGHT;

    // 1. 计算直方图
    for (uint32_t i = 0; i < total_pixels; i++) {
        histogram[input[i]]++;
    }

    // 2. 计算最佳阈值
    uint32_t sum = 0;
    for (uint16_t i = 0; i < 256; i++) {
        sum += i * histogram[i];  // 计算灰度总和
    }

    uint32_t w0 = 0;         // 前景像素数
    uint32_t sum0 = 0;       // 前景灰度和
    uint8_t threshold = 0;   // 最佳阈值
    float max_variance = 0;  // 最大类间方差

    for (uint16_t t = 0; t < 256; t++) {
        w0 += histogram[t];  // 前景像素累加
        if (w0 == 0)
            continue;

        uint32_t w1 = total_pixels - w0;  // 背景像素数
        if (w1 == 0)
            break;

        sum0 += t * histogram[t];

        // 计算均值
        float u0 = sum0 / (float)w0;          // 前景均值
        float u1 = (sum - sum0) / (float)w1;  // 背景均值

        // 计算类间方差
        float variance = w0 * w1 * (u0 - u1) * (u0 - u1);

        // 更新最大方差
        if (variance > max_variance) {
            max_variance = variance;
            threshold = t;
        }
    }

    // 3. 二值化
    for (uint32_t i = 0; i < total_pixels; i++) {
        output[i] = (input[i] > threshold) ? RGB565_WHITE : RGB565_BLACK;
    }
}

Point find_white_block_center(uint8_t* binary) {
    Point center = {-1, -1};  // 默认返回无效坐标
    uint16_t min_x = IMG_WIDTH;
    uint16_t max_x = 0;
    uint16_t min_y = IMG_HEIGHT;
    uint16_t max_y = 0;
    bool found = false;

    // 1. 找到第一个白色区块的边界
    for (uint16_t y = 0; y < IMG_HEIGHT; y++) {
        for (uint16_t x = 0; x < IMG_WIDTH; x++) {
            if (binary[y * IMG_WIDTH + x] == 0xFF) {  // 白色像素
                found = true;
                if (x < min_x)
                    min_x = x;
                if (x > max_x)
                    max_x = x;
                if (y < min_y)
                    min_y = y;
                if (y > max_y)
                    max_y = y;
            }
        }
    }

    // 2. 如果找到白色区块，计算中心点
    if (found) {
        center.x = (min_x + max_x) / 2;
        center.y = (min_y + max_y) / 2;
    }

    return center;
}

void draw_cross(uint8_t* img, Point center, uint8_t size, uint8_t color) {
    // 检查中心点是否有效
    if (center.x < 0 || center.x >= IMG_WIDTH || center.y < 0 ||
        center.y >= IMG_HEIGHT) {
        return;
    }

    if (size == (uint8_t)-1) {
        // 当 size 为 -1 时,填充整行和整列
        // 填充水平线
        for (int16_t x = 0; x < IMG_WIDTH; x++) {
            img[center.y * IMG_WIDTH + x] = color;
        }
        // 填充垂直线
        for (int16_t y = 0; y < IMG_HEIGHT; y++) {
            img[y * IMG_WIDTH + center.x] = color;
        }
        return;
    }

    // 画水平线
    int16_t start_x = center.x - size;
    int16_t end_x = center.x + size;
    if (start_x < 0)
        start_x = 0;
    if (end_x >= IMG_WIDTH)
        end_x = IMG_WIDTH - 1;

    for (int16_t x = start_x; x <= end_x; x++) {
        img[center.y * IMG_WIDTH + x] = color;
    }

    // 画垂直线
    int16_t start_y = center.y - size;
    int16_t end_y = center.y + size;
    if (start_y < 0)
        start_y = 0;
    if (end_y >= IMG_HEIGHT)
        end_y = IMG_HEIGHT - 1;

    for (int16_t y = start_y; y <= end_y; y++) {
        img[y * IMG_WIDTH + center.x] = color;
    }
}

// 种子填充法进行连通域标记
static void flood_fill(uint8_t* binary,
                       uint16_t x,
                       uint16_t y,
                       uint8_t label,
                       Region* region) {
    // 使用栈替代递归，防止栈溢出
    static int16_t stack_x[STACK_SIZE];
    static int16_t stack_y[STACK_SIZE];
    int16_t stack_pos = 0;

    // 将初始点压入栈
    stack_x[stack_pos] = x;
    stack_y[stack_pos] = y;
    stack_pos++;

    while (stack_pos > 0) {
        // 弹出一个点
        stack_pos--;
        x = stack_x[stack_pos];
        y = stack_y[stack_pos];

        // 如果不是白点，跳过
        if (x < 0 || x >= IMG_WIDTH || y < 0 || y >= IMG_HEIGHT ||
            binary[y * IMG_WIDTH + x] != 0xFF) {
            continue;
        }

        // 标记当前点
        binary[y * IMG_WIDTH + x] = label;

        // 更新区域信息
        if (x < region->min_x)
            region->min_x = x;
        if (x > region->max_x)
            region->max_x = x;
        if (y < region->min_y)
            region->min_y = y;
        if (y > region->max_y)
            region->max_y = y;
        region->area++;

        // 将四邻域点压入栈
        if (stack_pos < STACK_SIZE - 4) {
            stack_x[stack_pos] = x;
            stack_y[stack_pos] = y - 1;
            stack_pos++;

            stack_x[stack_pos] = x + 1;
            stack_y[stack_pos] = y;
            stack_pos++;

            stack_x[stack_pos] = x;
            stack_y[stack_pos] = y + 1;
            stack_pos++;

            stack_x[stack_pos] = x - 1;
            stack_y[stack_pos] = y;
            stack_pos++;
        }
    }
}

Point find_largest_white_region_center(uint8_t* binary) {
    static Region regions[MAX_REGIONS];               // 区域信息数组
    static uint8_t temp_img[IMG_WIDTH * IMG_HEIGHT];  // 临时图像
    uint8_t region_count = 0;                         // 区域计数
    Point center = {-1, -1};                          // 返回结果

    // 复制图像，避免修改原图
    memcpy(temp_img, binary, IMG_WIDTH * IMG_HEIGHT);

    // 扫描图像，进行连通域标记
    for (uint16_t y = 0; y < IMG_HEIGHT; y++) {
        for (uint16_t x = 0; x < IMG_WIDTH; x++) {
            if (temp_img[y * IMG_WIDTH + x] == 0xFF) {  // 找到白点
                if (region_count >= MAX_REGIONS)
                    break;  // 防止数组越界

                // 初始化新区域
                regions[region_count].min_x = x;
                regions[region_count].min_y = y;
                regions[region_count].max_x = x;
                regions[region_count].max_y = y;
                regions[region_count].area = 0;

                // 标记当前区域
                flood_fill(temp_img, x, y, region_count + 1,
                           &regions[region_count]);
                region_count++;
            }
        }
    }

    // 找出最大区域
    if (region_count > 0) {
        uint32_t max_area = 0;
        uint8_t max_idx = 0;

        for (uint8_t i = 0; i < region_count; i++) {
            if (regions[i].area > max_area) {
                max_area = regions[i].area;
                max_idx = i;
            }
        }

        // 计算中心点
        center.x = (regions[max_idx].min_x + regions[max_idx].max_x) / 2;
        center.y = (regions[max_idx].min_y + regions[max_idx].max_y) / 2;
    }

    return center;
}
#include "image.h"

// Otsu算法
float otsuThreshold(const uint8* image, uint16 width, uint16 height) {
    uint16 hist[256] = {0};
    float sum = width * height;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            hist[image[i * width + j]]++;
        }
    }

    float sumB = 0;
    uint16 wB = 0, wF = 0;

    float varMax = 0;
    float threshold = 0;

    for (uint16 t = 0; t < 256; t++) {
        wB += hist[t];  // Weight Background
        if (wB == 0)
            continue;

        wF = sum - wB;  // Weight Foreground
        if (wF == 0)
            break;

        sumB += (float)(t * hist[t]);

        float mB = sumB / wB;          // Mean Background
        float mF = (sum - sumB) / wF;  // Mean Foreground

        // Calculate Between Class Variance
        float varBetween = (float)wB * (float)wF * (mB - mF) * (mB - mF);

        // Check if new maximum found
        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = t;
        }
    }

    return threshold;
}

// 二值化图像
void binarizeImage(const uint8* image,
                   uint8* binaryImage,
                   uint16 width,
                   uint16 height,
                   double threshold) {
    for (uint16 i = 0; i < height; i++) {
        for (uint16 j = 0; j < width; j++) {
            if (image[i * width + j] >= threshold) {
                binaryImage[i * width + j] = 255;  // 白色
            } else {
                binaryImage[i * width + j] = 0;  // 黑色
            }
        }
    }
}

// 查找函数（路径压缩）
uint16 find(uint16* parent, uint16 x) {
    if (parent[x] != x)
        parent[x] = find(parent, parent[x]);  // 路径压缩
    return parent[x];
}

// 合并函数（按秩合并）
void unionSets(uint16* parent, uint16* rank, uint16 x, uint16 y) {
    uint16 rootX = find(parent, x);
    uint16 rootY = find(parent, y);
    if (rootX != rootY) {
        if (rank[rootX] < rank[rootY])
            parent[rootX] = rootY;
        else {
            parent[rootY] = rootX;
            if (rank[rootX] == rank[rootY])
                rank[rootX]++;
        }
    }
}

// 两遍扫描连通域标记算法
void twoPassLabeling(const uint8_t* binaryImage,
                     uint16 width,
                     uint16 height,
                     uint16* labeledImage) {
    uint16 labelCount = 1;
    uint16 maxLabels = width * height;
    uint16 parent[maxLabels];
    uint16 rank[maxLabels];

    for (uint16 i = 0; i < maxLabels; i++) {
        parent[i] = i;
        rank[i] = 0;
    }

    // 第一遍扫描：分配临时标签和等价关系
    for (uint16 y = 0; y < height; y++) {
        for (uint16 x = 0; x < width; x++) {
            if (binaryImage[y * width + x] == 255) {
                uint16 left = (x > 0 && labeledImage[y * width + (x - 1)] != 0)
                                  ? labeledImage[y * width + (x - 1)]
                                  : 0;
                uint16 above = (y > 0 && labeledImage[(y - 1) * width + x] != 0)
                                   ? labeledImage[(y - 1) * width + x]
                                   : 0;

                if (!left && !above) {
                    labeledImage[y * width + x] = labelCount++;
                } else if (left && !above) {
                    labeledImage[y * width + x] = left;
                } else if (!left && above) {
                    labeledImage[y * width + x] = above;
                } else {
                    labeledImage[y * width + x] = (left < above) ? left : above;
                    unionSets(parent, rank, left, above);
                }
            }
        }
    }

    // 第二遍扫描：统一等价标签
    for (uint16 y = 0; y < height; y++) {
        for (uint16 x = 0; x < width; x++) {
            if (labeledImage[y * width + x] != 0) {
                labeledImage[y * width + x] =
                    find(parent, labeledImage[y * width + x]);
            }
        }
    }
}

uint16 findLargestBlobLabel(uint16* labeledImage,
                            uint16 width,
                            uint16 height,
                            uint16* maxArea) {
    uint16 labelCount[64] = {0};
    *maxArea = 0;
    uint16 maxLabel = 0;

    for (uint16 y = 0; y < height; y++) {
        for (uint16 x = 0; x < width; x++) {
            uint16 label = labeledImage[y * width + x];
            if (label > 0) {
                labelCount[label]++;
            }
        }
    }

    for (uint16 i = 1; i < 64; i++) {
        if (labelCount[i] > *maxArea) {
            *maxArea = labelCount[i];
            maxLabel = i;
        }
    }

    return maxLabel;
}

void computeCentroid(uint16* labeledImage,
                     uint16 width,
                     uint16 height,
                     uint16 targetLabel,
                     uint16* out_x,
                     uint16* out_y) {
    long sumX = 0, sumY = 0, count = 0;

    for (uint16 y = 0; y < height; y++) {
        for (uint16 x = 0; x < width; x++) {
            if (labeledImage[y * width + x] == targetLabel) {
                sumX += x;
                sumY += y;
                count++;
            }
        }
    }

    if (count > 0) {
        *out_x = (uint16)(sumX / count);
        *out_y = (uint16)(sumY / count);
    } else {
        *out_x = -1;
        *out_y = -1;
    }
}

void drawRedPlus(uint8* image,
                 uint16 width,
                 uint16 height,
                 uint16 cx,
                 uint16 cy,
                 uint16 size) {
    for (uint16 i = -size; i <= size; i++) {
        // 横线
        if (cx + i >= 0 && cx + i < width)
            image[cy * width + (cx + i)] = 565;

        // 竖线
        if (cy + i >= 0 && cy + i < height)
            image[(cy + i) * width + cx] = 565;
    }
}

// void test() {
//     uint8_t binaryImage[HEIGHT][WIDTH];   // 输入的二值图像（0或255）
//     uint16_t rgb565Image[HEIGHT][WIDTH];  // 原始RGB565彩色图像

//     int labeledImage[HEIGHT][WIDTH];  // 标记后的图像（每个像素为区域标签）

//     // 第一步：执行两遍扫描连通域标记
//     twoPassLabeling((const uint8_t*)binaryImage, WIDTH, HEIGHT,
//                     (int*)labeledImage);

//     // 第二步：找出最大区域
//     int maxArea;
//     int maxLabel =
//         findLargestBlobLabel((int*)labeledImage, WIDTH, HEIGHT, &maxArea);

//     if (maxLabel == 0 || maxArea < 10) {
//         printf("未找到有效区域\n");
//         return -1;
//     }

//     printf("最大区域标签：%d，面积：%d\n", maxLabel, maxArea);

//     // 第三步：计算质心
//     int cx, cy;
//     computeCentroid((int*)labeledImage, WIDTH, HEIGHT, maxLabel, &cx, &cy);

//     if (cx == -1 || cy == -1) {
//         printf("无法计算质心\n");
//         return -1;
//     }

//     printf("质心坐标：(%d, %d)\n", cx, cy);

//     // 第四步：创建副本并在质心处画红十字
//     uint16_t markedImage[HEIGHT][WIDTH];
//     memcpy(markedImage, rgb565Image, sizeof(markedImage));

//     drawRedPlus((uint16_t*)markedImage, WIDTH, HEIGHT, cx, cy,
//                 5);  // 十字大小5x5

//     // 第五步：调用你的显示函数
//     tft180_show_rgb565_image(0, 0, (const uint16_t*)markedImage, WIDTH,
//     HEIGHT,
//                              WIDTH, HEIGHT, 1);
// }
#include "image.h"

void draw_cross(uint8 *img, Point center, uint8 size, uint8 color)
{
    // 检查中心点是否有效（包括-1,-1的无效标记）
    if (center.x < 0 || center.x >= IMG_WIDTH || center.y < 0 ||
        center.y >= IMG_HEIGHT)
    {
        return;
    }

    if (size == (uint8)-1)
    {
        // 当 size 为 -1 时,填充整行和整列
        // 填充水平线
        for (int16_t x = 0; x < IMG_WIDTH; x++)
        {
            img[center.y * IMG_WIDTH + x] = color;
        }
        // 填充垂直线
        for (int16_t y = 0; y < IMG_HEIGHT; y++)
        {
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

    for (int16_t x = start_x; x <= end_x; x++)
    {
        img[center.y * IMG_WIDTH + x] = color;
    }

    // 画垂直线
    int16_t start_y = center.y - size;
    int16_t end_y = center.y + size;
    if (start_y < 0)
        start_y = 0;
    if (end_y >= IMG_HEIGHT)
        end_y = IMG_HEIGHT - 1;

    for (int16_t y = start_y; y <= end_y; y++)
    {
        img[y * IMG_WIDTH + center.x] = color;
    }
}

void draw_Vmiddleline(uint8 *img, uint8 color)
{
    // 画垂直中线
    for (int16_t y = 0; y < IMG_HEIGHT; y++)
    {
        img[y * IMG_WIDTH + (IMG_WIDTH / 2)] = color;
    }
}

void draw_Hmiddleline(uint8 *img, uint8 color)
{
    // 画水平中线
    for (int16 x = 0; x < IMG_WIDTH; x++)
    {
        img[(IMG_HEIGHT / 2) * IMG_WIDTH + x] = color;
    }
}

void draw_Vline(uint8 *img, uint8 x, uint8 color)
{
    // 在指定位置画垂直线
    for (int16 y = 0; y < IMG_HEIGHT; y++)
    {
        img[y * IMG_WIDTH + x] = color;
    }
}

void draw_Hline(uint8 *img, uint8 y, uint8 color)
{
    // 在指定位置画水平线
    for (int16 x = 0; x < IMG_WIDTH; x++)
    {
        img[y * IMG_WIDTH + x] = color;
    }
}

void draw_rectangle(uint8 *img, uint16 x, uint16 y, uint16 width, uint16 height, uint8 color)
{
    uint16 x_end = x + width;
    uint16 y_end = y + height;

    // 边界检查
    if (x_end >= IMG_WIDTH) x_end = IMG_WIDTH - 1;
    if (y_end >= IMG_HEIGHT) y_end = IMG_HEIGHT - 1;

    // 绘制水平线
    for (uint16 i = x; i <= x_end; ++i) {
        img[y * IMG_WIDTH + i] = color;         // Top line
        img[y_end * IMG_WIDTH + i] = color;     // Bottom line
    }

    // 绘制垂直线
    for (uint16 i = y; i <= y_end; ++i) {
        img[i * IMG_WIDTH + x] = color;         // Left line
        img[i * IMG_WIDTH + x_end] = color;     // Right line
    }
}
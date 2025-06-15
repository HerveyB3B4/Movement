#include "image.h"
#include "detection.h"

static int16 img_target_error = 0;

static Point img_target_center = {-1, -1};
static uint16_t s_edge_map[MT9V03X_W][MT9V03X_H];

int16 get_img_target_error()
{
    return img_target_error;
}

void draw_cross(uint8_t *img, Point center, uint8_t size, uint8_t color)
{
    // 检查中心点是否有效
    if (center.x < 0 || center.x >= IMG_WIDTH || center.y < 0 ||
        center.y >= IMG_HEIGHT)
    {
        return;
    }

    if (size == (uint8_t)-1)
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

void draw_middleline(uint8_t *img, uint8_t color)
{
    // 画垂直中线
    for (int16_t y = 0; y < IMG_HEIGHT; y++)
    {
        img[y * IMG_WIDTH + (IMG_WIDTH / 2)] = color;
    }
}

void img_handler()
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        binary_otsu(mt9v03x_image, s_edge_map);
        img_target_center = find_white_center(s_edge_map, ALGORITHM_TWO_PASS);
        img_target_error = img_target_center.x - IMG_WIDTH / 2;
        draw_cross(s_edge_map, img_target_center, -1, RGB565_YELLOW);
        draw_middleline(s_edge_map, RGB565_YELLOW);
        lcd_show_image_mid(s_edge_map, MT9V03X_W, MT9V03X_H, 0);
    }
}
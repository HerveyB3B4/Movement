#include "image.h"
#include "detection.h"

static int16 img_target_error = 0;

static Point img_target_center = {-1, -1};
static uint16_t s_edge_map[MT9V03X_W][MT9V03X_H];

static uint32_t frame_count = 0;
static uint32_t last_time_ms = 0;
static float current_fps = 0.0f;

// 获取当前帧率
float get_img_fps()
{
    return current_fps;
}

int16 get_img_target_error()
{
    // 只有当找到有效目标时才计算偏差，否则保持原有偏差值
    // if (img_target_center.x >= 0 && img_target_center.y >= 0)
    // {
    //     img_target_error = img_target_center.x - IMG_WIDTH / 2;
    // }

    // 丢目标直接归零
    if (img_target_center.x >= 0 && img_target_center.y >= 0)
    {
        img_target_error = img_target_center.x - IMG_WIDTH / 2;
    }
    else
    {
        img_target_error = 0;
    }

    return img_target_error;
}

int16 get_img_target_distance()
{
    if (img_target_center.y < 0 || img_target_center.y >= IMG_HEIGHT)
    {
        return 0;
    }
    return img_target_center.y;
}

void draw_cross(uint8_t *img, Point center, uint8_t size, uint8_t color)
{
    // 检查中心点是否有效（包括-1,-1的无效标记）
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

void img_handler(uint8 lcd_flag)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;
        frame_count++;
        binary_otsu(mt9v03x_image, s_edge_map);
        img_target_center = find_white_center(s_edge_map, ALGORITHM_TWO_PASS);

        if (frame_count % 10 == 0)
        {
            uint32_t current_time = system_getval_ms();
            if (last_time_ms != 0)
            {
                uint32_t time_diff = current_time - last_time_ms;
                current_fps = 10000.0f / time_diff;
            }
            last_time_ms = current_time;
        }

        if (lcd_flag != 0)
        {
            draw_cross(s_edge_map, img_target_center, -1, RGB565_YELLOW);
            draw_middleline(s_edge_map, RGB565_YELLOW);
            lcd_show_image(s_edge_map, MT9V03X_W, MT9V03X_H, 0);
            lcd_show_int(0, 7, get_bottom_duty(), 4);
            // lcd_show_float(3, 7, get_img_fps(), 3, 0);
            lcd_show_int(8, 7, get_momentum_diff(), 4);
        }
    }
}
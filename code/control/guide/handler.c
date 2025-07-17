#include "guide.h"
#include "Attitude.h"
#include "handler.h"
#include "camera.h"

// main logic
static uint8 binary_front[IMG_WIDTH * IMG_HEIGHT];
static uint32 component_count = 0;
static Component_Info results[MAX_REGIONS];
static Run_State curr_state = STATE_SEARCHING;

// target handler
const uint32 LOSING_TIME_OUT = 200; // 200ms
static uint32 losing_cnt = 0;       // 连续丢失目标计数
static int8 direction_relation = 1; // 第二大的目标点在第一大目标点的左侧还是右侧，0左侧， 1右侧

// searching logic
const uint32 SEARCHING_SPEED = -16;
static int32 searching_turn_err = 0;
static int32 searching_last_turn_time = 0;

// tracking logic
const uint32 TRACKING_SPEED = -15;

// closing logic
const uint32 CLOSE_SPEED = -21;
const uint32 CLOSE_AREA_THRESHOLD = 450; // 接近目标的面积阈值
const uint32 CLOSE_H_THRESHOLD = 105;    // 接近目标的y坐标阈值

void handler_main()
{
    handler_single_camera();
    // 检查目标并处理转向逻辑
    handler_target();
    handler_execute();
}

void handler_single_camera()
{
    if (mt9v03x_finish_flag)
    {
        uint16 horizon_line = get_image_horizon();

        // binary_otsu(mt9v03x_image, binary_front);
        binary_threshold(mt9v03x_image, binary_front, g_binary_threshold_def);
        if (horizon_line > 0 && horizon_line < IMG_HEIGHT)
        {
            for (uint16 y = 0; y < horizon_line; y++)
            {
                for (uint16 x = 0; x < IMG_WIDTH; x++)
                {
                    binary_front[y * IMG_WIDTH + x] = RGB565_BLACK; // 设置为背景色
                }
            }
        }

        component_count = detection_find_components(binary_front, 0, results); // 前摄像头
        // 排序
        qsort(results, component_count, sizeof(Component_Info), compare_components);

        draw_cross(binary_front, results[0].center, 60, RGB565_WHITE);
        // lcd_show_int(4, 7, results[1].center.x, 3);
        // lcd_show_int(8, 7, results[1].center.y, 3);
        // lcd_show_int(12, 7, results[1].bbox.area, 3);

        draw_Hline(binary_front, horizon_line, RGB565_WHITE); // 在图像上画地平线
        lcd_show_image(binary_front, IMG_WIDTH, IMG_HEIGHT, 0);
        lcd_show_int(0, 7, searching_turn_err, 3);
        // reset status
        mt9v03x_finish_flag = 0;
    }
}

void handler_target()
{
    // 更新狀態
    // 连续丢失目标判定丢失
    if ((results[0].bbox.area == 0 && results[0].center.x == 0 && results[0].center.y == 0) || component_count == 0)
    {
        losing_cnt++;
        if (losing_cnt >= LOSING_TIME_OUT)
        {
            if (curr_state != STATE_SEARCHING)
            {
                curr_state = STATE_SEARCHING;
            }
        }
    }
    else
    {
        losing_cnt = 0;
        curr_state = STATE_TRACKING; // 有目标时，状态切换到跟踪
        if (results[0].bbox.area > CLOSE_AREA_THRESHOLD || results[0].center.y > CLOSE_H_THRESHOLD)
        {
            curr_state = STATE_CLOSE; // 接近目标
        }
    }
}

void handler_searching()
{
    searching_last_turn_time = system_getval_ms();

    if (searching_last_turn_time + 1000 >= system_getval_ms())
    {
        if (direction_relation == 0)
        {
            searching_turn_err -= 50; // 左侧目标，减少转向误差
        }
        else
        {
            searching_turn_err += 50; // 右侧目标，增加转向误差
        }
        searching_last_turn_time = system_getval_ms();
    }
    guide_set_target_vel(SEARCHING_SPEED);
    guide_set_target_turn(searching_turn_err);
}

void handler_tracking()
{
    searching_turn_err = 0;
    guide_set_target_vel(TRACKING_SPEED);
    guide_set_target_turn((results[0].center.x - IMG_WIDTH / 2) * 1.5);

    // 记录第二大的点和最大点的相对位置，决定searching方向
    if (component_count > 1)
    {
        direction_relation = (results[1].center.x < results[0].center.x) ? 0 : 1;
    }
}

void handler_close()
{
    searching_turn_err = 0;
    guide_set_target_vel(CLOSE_SPEED);
    guide_set_target_turn(results[0].center.x - IMG_WIDTH / 2);
    // 记录第二大的点和最大点的相对位置，决定searching方向
    if (component_count > 1)
    {
        direction_relation = (results[1].center.x < results[0].center.x) ? 0 : 1;
    }
}

void handler_execute()
{
    switch (curr_state)
    {
    case STATE_SEARCHING:
        handler_searching();
        break;

        // case STATE_TURNING:
        //     guide_set_target_vel(TURNING_SPEED);
        //     guide_set_target_turn(TURNING_TURN_ERR);
        //     break;

    case STATE_TRACKING:
        handler_tracking();
        break;

    case STATE_CLOSE:
        handler_close();
        break;

    default:
        break;
    }
}
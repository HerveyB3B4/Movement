#include "state_machine.h"
#include "detection.h"
#include "guide.h"
#include "Attitude.h"
#include "camera.h"

const uint32 BOUNDARY_AREA = 450; // 临界面积大小
const uint32 BOUNDARY_H = 105;    // 临界y坐标

const int32 TRACKING_SPEED = -34;
const int32 SEARCHING_SPEED = -10;
const int32 TURNING_SPEED = -20;
const int32 CLOSE_SPEED = -6;
const uint32 SEARCHING_TURN_ERROR = 50;

const uint32 NO_TARGET_TIMEOUT = 50; // 无目标超时计数阈值

static Run_State curr_state;
static Camera_Mode curr_camera_mode = SINGLE_CAMERA;
static uint8 front_img = 0;
static Component_Info results[MAX_REGIONS];
static uint32 component_count = 0;

// 转向相关参数
static uint32 no_target_counter = 0; // 无目标计数器

// #pragma section all "cpu1_dsram"
static uint8 binary_front[IMG_WIDTH * IMG_HEIGHT];
static uint8 binary_rear[IMG_WIDTH * IMG_HEIGHT];
// #pragma section all restore

static void state_machine_single_handler(void);
static void state_machine_dual_handler(void);
static void state_machine_check_targets(void);

void state_machine_init(Camera_Mode mode)
{
    curr_state = STATE_SEARCHING;
    curr_camera_mode = mode;
    detection_init(ALGORITHM_FLOOD_FILL);
    no_target_counter = 0;
}

void state_machine_imghandler()
{
    switch (curr_camera_mode)
    {
    case SINGLE_CAMERA:
        state_machine_single_handler();
        break;
    case DUAL_CAMERA:
        state_machine_dual_handler();
        break;
    default:
        break;
    }

    // 检查目标并处理转向逻辑
    state_machine_check_targets();
    // lcd_show_int(0, 7, curr_state, 1);
}

static void state_machine_single_handler()
{
    if (mt9v03x_finish_flag)
    {
        uint16 horizon_line = get_image_horizon();

        // binary_otsu(mt9v03x_image, binary_front);
        binary_threshold(mt9v03x_image, binary_front, g_binary_threshold_def);
        // 将地平线以上部分清零
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
        draw_Hline(binary_front, horizon_line, RGB565_WHITE); // 在图像上画地平线
        lcd_show_int(0, 7, results[0].center.x, 3);
        lcd_show_int(4, 7, results[0].center.y, 3);
        lcd_show_int(8, 7, results[0].bbox.min_x, 3);
        lcd_show_int(12, 7, results[0].bbox.max_x, 3);
        lcd_show_image(binary_front, IMG_WIDTH, IMG_HEIGHT, 0);

        // reset status
        mt9v03x_finish_flag = 0;
    }
}

static void state_machine_dual_handler()
{
    if (mt9v03x_finish_flag && mt9v03x2_finish_flag)
    {
        // 前摄像头
        binary_otsu(mt9v03x_image, binary_front);
        uint16 front_count = detection_find_components(binary_front, 0, results);

        // 后摄像头
        binary_otsu(mt9v03x2_image, binary_rear);
        uint16 rear_count = 0;
        if (front_count < MAX_REGIONS)
            rear_count = detection_find_components(binary_rear, 1, results + front_count);

        // 合并两个摄像头获取到的结果
        component_count = front_count + rear_count;
        qsort(results, component_count, sizeof(Component_Info), compare_components);

        // reset status
        mt9v03x_finish_flag = 0;
        mt9v03x2_finish_flag = 0;
    }
}

static void state_machine_check_targets(void)
{
    if (component_count == 0 ||
        (component_count > 0 && results[0].center.x == -1 && results[0].center.y == -1) ||
        (results[0].center.x == 0 && results[0].center.y == 0) || // 添加检测坐标(0,0)的条件
        results[0].bbox.area <= 2)
    {
        // 如果没有检测到目标，增加无目标计数器
        no_target_counter++;

        if (no_target_counter >= NO_TARGET_TIMEOUT)
        {
            state_machine_set_state(STATE_SEARCHING);
            no_target_counter = 0; // 重置计数器
        }
    }
    else
    {
        no_target_counter = 0; // 重置计数器

        // 计算目标中心与图像中心的偏差
        int32 target_offset = results[0].center.x - IMG_WIDTH / 2;

        // 判断是否接近目标
        if (results[0].bbox.area >= BOUNDARY_AREA || results[0].center.y >= BOUNDARY_H)
        {
            // 已经接近目标，需要减速
            state_machine_set_state(STATE_CLOSE);
        }
        // 判断是否需要转向对准
        else if (abs(target_offset) > 15) // 偏差较大，需要调整转向
        {
            // 需要转向对准目标
            state_machine_set_state(STATE_TURNING);
        }
        else
        {
            // 目标已对准，加速前进追踪
            state_machine_set_state(STATE_TRACKING);
        }
    }
}

void state_machine_set_state(Run_State state)
{
    curr_state = state;
    switch (curr_state)
    {
    case STATE_TURNING:
        // 低速转向对准目标
        guide_set_target_vel(TURNING_SPEED); // 降低速度以便更精确地转向
        // 根据目标位置调整转向，偏差越大转向越大
        int32 turn_amount = results[0].center.x - IMG_WIDTH / 2;
        guide_set_target_turn(turn_amount);

        break;

    case STATE_TRACKING:
        // 转向对准后加速前进追踪目标
        guide_set_target_vel(TRACKING_SPEED); // 提高速度追踪目标
        // 保持小幅度的转向修正，确保目标居中
        guide_set_target_turn((results[0].center.x - IMG_WIDTH / 2) * 0.3);
        break;

    case STATE_CLOSE:
        guide_set_target_vel(CLOSE_SPEED); // 接近目标时减速
        int32 offset = results[0].center.x - IMG_WIDTH / 2;
        if (abs(offset) > 20)
        {
            // 偏离较大时进行更强的转向修正
            guide_set_target_turn(offset * 0.8);
        }
        else
        {
            guide_set_target_turn(offset * 0.5);
        }
        break;

    case STATE_SEARCHING:
    default:
        // 低速转圈寻找目标
        guide_set_target_vel(10); // 搜索时保持适中速度
        // 使用Roll角度来确定转向方向，避免一直往一个方向转
        if (ROLL >= 0)
        {
            guide_set_target_turn(SEARCHING_TURN_ERROR);
        }
        else
        {
            guide_set_target_turn(-SEARCHING_TURN_ERROR);
        }
        break;
    }
}

// 获取当前检测到的目标数量
uint32 state_machine_get_component_count(void)
{
    return component_count;
}

// 获取当前检测到的目标信息
Component_Info *state_machine_get_components(void)
{
    return results;
}

// 获取当前运行状态
Run_State state_machine_get_state(void)
{
    return curr_state;
}

uint8 *state_machine_get_front_img(void)
{
    return binary_front;
}
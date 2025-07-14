#include "state_machine.h"
#include "detection.h"
#include "guide.h"
#include "Attitude.h"

const uint32 BOUNDARY_AREA = 20; // 临界面积大小，确定是否进入下一个状态
const uint32 BOUNDARY_H = 80;    // 临界高度，确定是否进入下一个状态

const int32 TRACKING_SPEED = -34;
const int32 SEARCHING_SPEED = -10;
const int32 TURNING_SPEED = -20;
const int32 CLOSE_SPEED = -6;

static Run_State curr_state;
static Camera_Mode curr_camera_mode = SINGLE_CAMERA;
static uint8 front_img = 0;
static Component_Info results[MAX_REGIONS];
static uint32 component_count = 0;

// 转向相关参数
static int32 search_turn_speed = 50;  // 搜索时的转向速度
static uint32 no_target_timeout = 50; // 无目标超时计数阈值
static uint32 no_target_counter = 0;  // 无目标计数器

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
    detection_init(ALGORITHM_TWO_PASS);
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
}

static void state_machine_single_handler()
{
    if (mt9v03x_finish_flag)
    {
        binary_otsu(mt9v03x_image, binary_front);
        component_count = detection_find_components(binary_front, 0, results); // 前摄像头
        // 排序
        qsort(results, component_count, sizeof(Component_Info), compare_components);

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

// 检查目标并处理无目标时的转向逻辑
static void state_machine_check_targets(void)
{
    if (component_count == 0 ||
        (component_count > 0 && results[0].center.x == -1 && results[0].center.y == -1) ||
        results[0].bbox.area <= 5)
    {
        // 如果没有检测到目标，增加无目标计数器
        no_target_counter++;

        if (no_target_counter >= no_target_timeout)
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
        guide_set_target_vel(8); // 降低速度以便更精确地转向
        // 根据目标位置调整转向，偏差越大转向越大
        int32 turn_amount = results[0].center.x - IMG_WIDTH / 2;
        if (abs(turn_amount) > 40)
        {
            // 目标偏离较大，加大转向力度
            guide_set_target_turn(turn_amount * 1.2);
        }
        else
        {
            guide_set_target_turn(turn_amount);
        }
        break;

    case STATE_TRACKING:
        // 转向对准后加速前进追踪目标
        guide_set_target_vel(34); // 提高速度追踪目标
        // 保持小幅度的转向修正，确保目标居中
        guide_set_target_turn((results[0].center.x - IMG_WIDTH / 2) * 0.3);
        break;

    case STATE_CLOSE:
        // 接近目标时减速并根据目标位置进行更精确的控制
        guide_set_target_vel(6); // 降低速度以便精确操作

        int32 offset = results[0].center.x - IMG_WIDTH / 2;
        if (abs(offset) > 20)
        {
            // 偏离较大时进行更强的转向修正
            guide_set_target_turn(offset * 0.8);
        }
        else
        {
            // 偏离较小时进行微调
            guide_set_target_turn(offset * 0.5);
        }
        break;

    case STATE_SEARCHING:
    default:
        // 搜索状态：低速转圈寻找目标
        guide_set_target_vel(10); // 搜索时保持适中速度
        // 使用Roll角度来确定转向方向，避免一直往一个方向转
        if (ROLL >= 0)
        {
            guide_set_target_turn(search_turn_speed);
        }
        else
        {
            guide_set_target_turn(-search_turn_speed);
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
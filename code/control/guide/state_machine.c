#include "state_machine.h"
#include "detection.h"
#include "guide.h" // 添加guide.h头文件以调用转向函数

static Run_State curr_state;
static Camera_Mode curr_camera_mode = SINGLE_CAMERA;
static uint8 front_img = 0;
static Component_Info results[MAX_REGIONS]; // TODO:需要确认大小
static uint32 component_count = 0;

// 转向相关参数
static int32 search_turn_speed = 1500; // 搜索时的转向速度
static uint32 no_target_timeout = 50;  // 无目标超时计数阈值
static uint32 no_target_counter = 0;   // 无目标计数器

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

// 预留接口
void state_machine_set_runstate(Run_State state)
{
    curr_state = state;
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
    if (component_count == 0)
    {
        // 没有找到目标，增加计数器
        no_target_counter++;

        // 如果超过一定时间没有目标，开始转向搜索
        if (no_target_counter >= no_target_timeout)
        {
            switch (curr_state)
            {
            case STATE_SEARCHING:
                // 在搜索状态下，设置转向寻找目标
                guide_set_target_turn(search_turn_speed);
                guide_set_target_vel(0); // 停止前进，只转向
                break;
            case STATE_APPROACHING:
            case STATE_CIRCLING:
                // 在接近或环绕状态下丢失目标，回到搜索状态并转向
                curr_state = STATE_SEARCHING;
                guide_set_target_turn(search_turn_speed);
                guide_set_target_vel(0);
                break;
            default:
                break;
            }
        }
    }
    else
    {
        // 找到目标，重置计数器并停止搜索转向
        no_target_counter = 0;

        // 如果之前在搜索状态，现在找到目标了，可以切换到接近状态
        if (curr_state == STATE_SEARCHING)
        {
            curr_state = STATE_APPROACHING;
            guide_set_target_turn(0); // 停止搜索转向
        }
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

// 设置搜索转向速度
void state_machine_set_search_turn_speed(int32 speed)
{
    search_turn_speed = speed;
}

// 设置无目标超时阈值
void state_machine_set_no_target_timeout(uint32 timeout)
{
    no_target_timeout = timeout;
}
#include "state_machine.h"
#include "detection.h"
#include "guide.h"
#include "Attitude.h"
#include "camera.h"

const uint32 BOUNDARY_AREA = 450; // 临界面积大小
const uint32 BOUNDARY_H = 105;    // 临界y坐标

const int32 TRACKING_SPEED = -34;
const int32 SEARCHING_SPEED = -20;
const int32 TURNING_SPEED = -20;
const int32 CLOSE_SPEED = -15;
const uint32 SEARCHING_TURN_ERROR = 60;

const uint32 NO_TARGET_TIMEOUT = 50;   // 无目标超时计数阈值
const uint32 TARGET_LOST_TIMEOUT = 15; // 目标丢失超时阈值（更短）
const uint32 STABLE_TARGET_FRAMES = 3; // 稳定目标需要的连续帧数
const uint32 MAX_TARGET_DISTANCE = 50; // 目标最大移动距离
const uint32 MIN_VALID_AREA = 5;       // 最小有效目标面积

// 目标跟踪历史信息
typedef struct
{
    Component_Info target;
    bool is_valid;
    uint32 stable_frames; // 连续稳定帧数
    uint32 lost_frames;   // 丢失帧数
} Target_Track;

static Run_State curr_state;
static Camera_Mode curr_camera_mode = SINGLE_CAMERA;
static uint8 front_img = 0;
static Component_Info results[MAX_REGIONS];
static uint32 component_count = 0;

// 转向相关参数
static uint32 no_target_counter = 0; // 无目标计数器

// 目标跟踪相关
static Target_Track tracked_target = {0};
static Component_Info predicted_target = {0};
static bool has_predicted_target = false;

// #pragma section all "cpu1_dsram"
static uint8 binary_front[IMG_WIDTH * IMG_HEIGHT];
static uint8 binary_rear[IMG_WIDTH * IMG_HEIGHT];
// #pragma section all restore

static void state_machine_single_handler(void);
static void state_machine_dual_handler(void);
static void state_machine_handler(void);
static void state_machine_execute(void);
static bool is_target_similar(const Component_Info *target1, const Component_Info *target2);
static void update_target_tracking(void);
static Component_Info *find_best_target(void);

void state_machine_init(Camera_Mode mode)
{
    curr_state = STATE_SEARCHING;
    curr_camera_mode = mode;
    detection_init(ALGORITHM_FLOOD_FILL);
    no_target_counter = 0;

    // 初始化目标跟踪相关变量
    tracked_target.is_valid = false;
    tracked_target.stable_frames = 0;
    tracked_target.lost_frames = 0;
    has_predicted_target = false;
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
    state_machine_handler();
    state_machine_execute();

    lcd_show_int(0, 7, curr_state, 1);
}

static void state_machine_single_handler()
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
        lcd_show_int(4, 7, results[0].center.x, 3);
        lcd_show_int(8, 7, results[0].center.y, 3);

        draw_Hline(binary_front, horizon_line, RGB565_WHITE); // 在图像上画地平线
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

static void state_machine_handler(void)
{
    // 更新目标跟踪信息
    update_target_tracking();

    // 改进的目标检测逻辑 - 使用跟踪信息
    bool has_valid_target = false;
    Component_Info *current_target = NULL;

    // 优先使用跟踪的目标
    if (tracked_target.is_valid && tracked_target.stable_frames >= STABLE_TARGET_FRAMES)
    {
        has_valid_target = true;
        current_target = &tracked_target.target;
        no_target_counter = 0; // 重置计数器
    }
    // 如果没有稳定的跟踪目标，尝试使用检测到的目标
    else if (component_count > 0)
    {
        // 使用智能目标选择
        Component_Info *best_target = find_best_target();
        if (best_target && best_target->center.x > 0 && best_target->center.y > 0 &&
            best_target->center.x != -1 && best_target->center.y != -1 &&
            best_target->bbox.area > MIN_VALID_AREA)
        {
            has_valid_target = true;
            current_target = best_target;
            no_target_counter = 0; // 重置计数器
        }
    }
    // 如果都没有，但有预测目标，可以短暂使用预测目标
    else if (has_predicted_target && tracked_target.lost_frames < TARGET_LOST_TIMEOUT / 2)
    {
        has_valid_target = true;
        current_target = &predicted_target;
        // 不重置no_target_counter，让它继续累积
    }

    if (!has_valid_target)
    {
        // 如果没有检测到有效目标，增加无目标计数器
        no_target_counter++;

        // 只有在超时后才切换到搜索状态，避免频繁切换
        if (no_target_counter >= NO_TARGET_TIMEOUT && curr_state != STATE_SEARCHING)
        {
            state_machine_set_state(STATE_SEARCHING);
            no_target_counter = 0; // 重置计数器
        }
    }
    else
    {
        // 计算目标中心与图像中心的偏差
        int32 target_offset = current_target->center.x - IMG_WIDTH / 2;

        // 状态切换逻辑 - 添加状态稳定性检查
        Run_State next_state = curr_state;

        // 判断是否接近目标
        if (current_target->bbox.area >= BOUNDARY_AREA || current_target->center.y >= BOUNDARY_H)
        {
            next_state = STATE_CLOSE;
        }
        // 判断是否需要转向对准
        else if (abs(target_offset) > 15) // 偏差较大，需要调整转向
        {
            next_state = STATE_TURNING;
        }
        else
        {
            // 目标已对准，加速前进追踪
            next_state = STATE_TRACKING;
        }

        // 只有当状态确实需要改变时才切换
        if (next_state != curr_state)
        {
            state_machine_set_state(next_state);
        }

        // 更新results[0]以便其他函数使用
        if (current_target != &results[0])
        {
            results[0] = *current_target;
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
        if (component_count > 0 && results[0].center.x > 0)
        {
            int32 turn_amount = results[0].center.x - IMG_WIDTH / 2;
            guide_set_target_turn(turn_amount * 1.5);
        }
        else
        {
            guide_set_target_turn(0); // 无目标时不转向
        }
        break;

    case STATE_TRACKING:
        // 转向对准后加速前进追踪目标
        guide_set_target_vel(TRACKING_SPEED); // 提高速度追踪目标
        // 保持小幅度的转向修正，确保目标居中
        if (component_count > 0 && results[0].center.x > 0)
        {
            guide_set_target_turn((results[0].center.x - IMG_WIDTH / 2));
        }
        else
        {
            guide_set_target_turn(0);
        }
        break;

    case STATE_CLOSE:
        guide_set_target_vel(CLOSE_SPEED); // 接近目标时减速
        if (component_count > 0 && results[0].center.x > 0)
        {
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
        }
        else
        {
            guide_set_target_turn(0);
        }
        break;

    case STATE_SEARCHING:
    default:
        // 低速转圈寻找目标
        guide_set_target_vel(SEARCHING_SPEED); // 搜索时保持适中速度
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

static void state_machine_execute(void)
{
    switch (curr_state)
    {
    case STATE_TURNING:
        guide_set_target_vel(TURNING_SPEED);
        if (component_count > 0 && results[0].center.x > 0)
        {
            int32 turn_amount = results[0].center.x - IMG_WIDTH / 2;
            guide_set_target_turn(turn_amount);
        }
        else
        {
            guide_set_target_turn(0);
        }
        break;

    case STATE_TRACKING:
        // 转向对准后加速前进追踪目标
        guide_set_target_vel(TRACKING_SPEED);
        if (component_count > 0 && results[0].center.x > 0)
        {
            guide_set_target_turn((results[0].center.x - IMG_WIDTH / 2) * 0.3);
        }
        else
        {
            guide_set_target_turn(0);
        }
        break;

    case STATE_CLOSE:
        guide_set_target_vel(CLOSE_SPEED);
        if (component_count > 0 && results[0].center.x > 0)
        {
            int32 offset = results[0].center.x - IMG_WIDTH / 2;
            if (abs(offset) > 20)
            {
                guide_set_target_turn(offset * 0.8);
            }
            else
            {
                guide_set_target_turn(offset * 0.5);
            }
        }
        else
        {
            guide_set_target_turn(0);
        }
        break;

    case STATE_SEARCHING:
    default:
        // 低速转圈寻找目标
        guide_set_target_vel(SEARCHING_SPEED);
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

static bool is_target_similar(const Component_Info *target1, const Component_Info *target2)
{
    // 简单的相似性判断，您可以根据需要改进
    if (abs(target1->center.x - target2->center.x) < MAX_TARGET_DISTANCE &&
        abs(target1->center.y - target2->center.y) < MAX_TARGET_DISTANCE &&
        abs((int32)target1->bbox.area - (int32)target2->bbox.area) < (target1->bbox.area / 4))
    {
        return true;
    }
    return false;
}

static void update_target_tracking(void)
{
    if (component_count > 0 && results[0].center.x > 0)
    {
        if (tracked_target.is_valid)
        {
            // 目标跟踪中，检查目标是否仍然有效
            if (is_target_similar(&tracked_target.target, &results[0]))
            {
                // 目标相似，更新跟踪信息
                tracked_target.target = results[0];
                tracked_target.stable_frames++;
                tracked_target.lost_frames = 0;
            }
            else
            {
                // 目标不再相似，增加丢失帧数
                tracked_target.lost_frames++;
            }

            // 超过阈值则认为目标丢失
            if (tracked_target.lost_frames >= TARGET_LOST_TIMEOUT)
            {
                tracked_target.is_valid = false;
            }
        }
        else
        {
            // 目标未被跟踪，尝试开始跟踪
            if (results[0].bbox.area > MIN_VALID_AREA)
            {
                tracked_target.target = results[0];
                tracked_target.is_valid = true;
                tracked_target.stable_frames = 1;
                tracked_target.lost_frames = 0;
            }
        }
    }
    else
    {
        // 没有检测到目标，标记为无效
        tracked_target.is_valid = false;
    }

    // 预测目标位置
    if (tracked_target.is_valid && tracked_target.stable_frames >= STABLE_TARGET_FRAMES)
    {
        predicted_target = tracked_target.target;
        has_predicted_target = true;
    }
    else
    {
        has_predicted_target = false;
    }
}

static Component_Info *find_best_target(void)
{
    Component_Info *best_target = NULL;
    uint32 best_area = 0;

    for (uint32 i = 0; i < component_count; i++)
    {
        if (results[i].bbox.area > best_area)
        {
            best_area = results[i].bbox.area;
            best_target = &results[i];
        }
    }

    return best_target;
}
#include "state_machine.h"
#include "detection.h"
#include "guide.h"
#include "Attitude.h"
#include "camera.h"

const uint32 BOUNDARY_AREA = 450; // 临界面积大小
const uint32 BOUNDARY_H = 105;    // 临界y坐标

const int32 TRACKING_SPEED = -20;
const int32 SEARCHING_SPEED = 0;
const int32 TURNING_SPEED = -20;
const int32 CLOSE_SPEED = -15;
const uint32 SEARCHING_TURN_ERROR = 120;

const uint32 NO_TARGET_TIMEOUT = 50;   // 无目标超时计数阈值
const uint32 TARGET_LOST_TIMEOUT = 15; // 目标丢失超时阈值（更短）
const uint32 STABLE_TARGET_FRAMES = 3; // 稳定目标需要的连续帧数
const uint32 MAX_TARGET_DISTANCE = 50; // 目标最大移动距离

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

// 函数声明
static void handle_single_camera(void);
static void handle_dual_camera(void);
static void handle_target_logic(void);
static void execute_state_control(void);
static bool is_similar_target(const Component_Info *target1, const Component_Info *target2);
static void update_tracking(void);
static Pass_Strategy calc_pass_strategy(void);
static int32 get_offset_with_strategy(const Component_Info *target, Pass_Strategy strategy);

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
        handle_single_camera();
        break;
    case DUAL_CAMERA:
        handle_dual_camera();
        break;
    default:
        break;
    }

    // 检查目标并处理转向逻辑
    handle_target_logic();
    execute_state_control();

    lcd_show_int(0, 7, curr_state, 1);
    lcd_show_int(5, 7, get_guide_target_vel(), 3);
}

static void handle_single_camera()
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

        // reset status
        mt9v03x_finish_flag = 0;
    }
}

static void handle_dual_camera()
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

static void handle_target_logic(void)
{
    // 更新目标跟踪信息
    update_tracking();

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
        // 直接使用已排序的第一个目标（最大的目标）
        if (results[0].center.x > 0 && results[0].center.y > 0 && results[0].bbox.area > 0)
        {
            has_valid_target = true;
            current_target = &results[0];
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
        // 找到目标后停止智能搜索
        // 计算通过策略
        Pass_Strategy strategy = calc_pass_strategy();

        // 根据策略计算目标偏移
        int32 target_offset = get_offset_with_strategy(current_target, strategy);

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
        guide_set_target_vel(TURNING_SPEED);
        break;

    case STATE_TRACKING:
        // 转向对准后加速前进追踪目标
        guide_set_target_vel(TRACKING_SPEED);
        break;

    case STATE_CLOSE:
        guide_set_target_vel(CLOSE_SPEED);
        break;

    case STATE_SEARCHING:
    default:
        // 低速转圈寻找目标
        guide_set_target_vel(SEARCHING_SPEED);
        break;
    }
}

static void execute_state_control(void)
{
    // 计算通过策略
    Pass_Strategy strategy = calc_pass_strategy();

    // 基础速度
    int32 target_speed = TRACKING_SPEED;

    // 检查是否有大目标或接近目标，需要减速
    if (component_count > 0 &&
        (results[0].bbox.area >= 480 || results[0].center.y > 108))
    {
        target_speed = -10; // 通过目标时减速
    }

    switch (curr_state)
    {
    case STATE_TURNING:
        guide_set_target_vel(target_speed);
        if (component_count > 0 && results[0].center.x > 0)
        {
            int32 turn_amount = get_offset_with_strategy(&results[0], strategy);
            guide_set_target_turn(turn_amount * 1.5);
        }
        else
        {
            guide_set_target_turn(0);
        }
        break;

    case STATE_TRACKING:
        guide_set_target_vel(target_speed);
        if (component_count > 0 && results[0].center.x > 0)
        {
            int32 turn_amount = get_offset_with_strategy(&results[0], strategy);
            guide_set_target_turn(turn_amount);
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
            int32 offset = get_offset_with_strategy(&results[0], strategy);
            guide_set_target_turn(offset);
        }
        else
        {
            guide_set_target_turn(0);
        }
        break;

    case STATE_SEARCHING:
    default:
        guide_set_target_vel(SEARCHING_SPEED);

        // 智能搜索逻辑：如果有第二大目标信息，朝向它搜索
        if (component_count >= 2 && results[1].bbox.area > 0)
        {
            int32 search_direction = results[1].center.x - IMG_WIDTH / 2;
            // 添加偏移避免直接朝向目标
            search_direction += (search_direction > 0) ? -30 : 30;
            guide_set_target_turn(search_direction);
        }
        else
        {
            // 默认搜索方向
            if (ROLL >= 0)
            {
                guide_set_target_turn(SEARCHING_TURN_ERROR);
            }
            else
            {
                guide_set_target_turn(-SEARCHING_TURN_ERROR);
            }
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

static bool is_similar_target(const Component_Info *target1, const Component_Info *target2)
{
    // 简单的相似性判断
    if (abs(target1->center.x - target2->center.x) < MAX_TARGET_DISTANCE &&
        abs(target1->center.y - target2->center.y) < MAX_TARGET_DISTANCE &&
        abs((int32)target1->bbox.area - (int32)target2->bbox.area) < (target1->bbox.area / 4))
    {
        return true;
    }
    return false;
}

static void update_tracking(void)
{
    // 首先检查是否有有效的检测结果
    bool has_valid_detection = false;
    Component_Info *current_detection = NULL;

    if (component_count > 0)
    {
        // 直接使用已排序的第一个目标（最大的目标）
        if (results[0].center.x != 0 && results[0].center.y != 0 && results[0].bbox.area > 0)
        {
            has_valid_detection = true;
            current_detection = &results[0];
        }
    }

    if (has_valid_detection)
    {
        if (tracked_target.is_valid)
        {
            // 目标跟踪中，检查目标是否仍然有效
            if (is_similar_target(&tracked_target.target, current_detection))
            {
                // 目标相似，更新跟踪信息
                tracked_target.target = *current_detection;
                tracked_target.stable_frames++;
                tracked_target.lost_frames = 0;
            }
            else
            {
                // 目标不再相似，增加丢失帧数
                tracked_target.lost_frames++;

                // 如果丢失帧数不太多，仍然保持跟踪状态
                if (tracked_target.lost_frames < TARGET_LOST_TIMEOUT)
                {
                    // 保持原有目标信息，但不更新位置
                    // 这样可以避免频繁闪烁
                }
                else
                {
                    // 超过阈值，重新开始跟踪新目标
                    tracked_target.target = *current_detection;
                    tracked_target.is_valid = true;
                    tracked_target.stable_frames = 1;
                    tracked_target.lost_frames = 0;
                }
            }
        }
        else
        {
            // 目标未被跟踪，尝试开始跟踪
            if (current_detection->bbox.area > 0)
            {
                tracked_target.target = *current_detection;
                tracked_target.is_valid = true;
                tracked_target.stable_frames = 1;
                tracked_target.lost_frames = 0;
            }
        }
    }
    else
    {
        // 没有检测到有效目标
        if (tracked_target.is_valid)
        {
            tracked_target.lost_frames++;

            // 只有在连续丢失较长时间后才标记为无效
            if (tracked_target.lost_frames >= TARGET_LOST_TIMEOUT)
            {
                tracked_target.is_valid = false;
                tracked_target.stable_frames = 0;
            }
        }
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

static Pass_Strategy calc_pass_strategy(void)
{
    if (component_count < 2)
    {
        return PASS_STRATEGY_CENTER; // 只有一个目标时直接去灭灯
    }

    // 获取第一大和第二大目标
    Component_Info *first_target = &results[0];  // 最大的目标（主要目标）
    Component_Info *second_target = &results[1]; // 第二大的目标

    // 检查第二大目标是否足够大，值得考虑
    if (second_target->bbox.area <= 0)
    {
        return PASS_STRATEGY_CENTER; // 第二个目标太小，忽略
    }

    // 简单判断：第二大目标在第一大目标的哪一侧
    if (second_target->center.x < first_target->center.x)
    {
        // 第二大目标在左侧，从右侧通过更好
        return PASS_STRATEGY_RIGHT;
    }
    else
    {
        // 第二大目标在右侧，从左侧通过更好
        return PASS_STRATEGY_LEFT;
    }
}

static int32 get_offset_with_strategy(const Component_Info *target, Pass_Strategy strategy)
{
    switch (strategy)
    {
    case PASS_STRATEGY_LEFT:
        // 从左侧通过：目标位置向左偏移
        return target->bbox.min_x - 10;

    case PASS_STRATEGY_RIGHT:
        // 从右侧通过：目标位置向右偏移
        return target->bbox.max_x + 10;

    case PASS_STRATEGY_CENTER:
    default:
        // 直接瞄准目标中心去灭灯
        return target->center.x;
    }
}
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

// 通过点后临时减速参数
const int32 TEMP_SLOW_SPEED = 10;        // 临时减速后的速度
const uint32 SLOW_DURATION_FRAMES = -20; // 减速持续帧数（约1秒）

const uint32 PASSED_TARGET_AREA_THRESHOLD = 480; // 判定通过目标的面积阈值

const uint32 NO_TARGET_TIMEOUT = 50;   // 无目标超时计数阈值
const uint32 TARGET_LOST_TIMEOUT = 15; // 目标丢失超时阈值（更短）
const uint32 STABLE_TARGET_FRAMES = 3; // 稳定目标需要的连续帧数
const uint32 MAX_TARGET_DISTANCE = 50; // 目标最大移动距离

// 侧通过策略参数
const uint32 SIDE_PASS_OFFSET = 30; // 侧通过时的偏移量

// 新增：智能搜索相关参数
const uint32 SMART_SEARCH_DURATION = 30;  // 智能搜索持续帧数
const int32 SMART_SEARCH_TURN_SPEED = 80; // 智能搜索时的转向速度

// 目标跟踪历史信息
typedef struct
{
    Component_Info target;
    bool is_valid;
    uint32 stable_frames; // 连续稳定帧数
    uint32 lost_frames;   // 丢失帧数
} Target_Track;

// 新增：临时减速状态结构
typedef struct
{
    bool is_active;          // 是否正在减速
    uint32 remaining_frames; // 剩余减速帧数
} Temp_Slow_State;

// 新增：已通过目标记录结构
typedef struct
{
    Point position;   // 目标位置
    uint32 area;      // 目标面积
    bool is_recorded; // 是否已记录
} Passed_Target;

// 新增：智能搜索状态结构
typedef struct
{
    bool is_active;                    // 是否正在智能搜索
    uint32 remaining_frames;           // 剩余搜索帧数
    int32 search_direction;            // 搜索方向 (负数左转，正数右转)
    Component_Info last_second_target; // 上次的第二大目标
    bool has_last_second_target;       // 是否有上次第二大目标记录
} Smart_Search_State;

#define MAX_PASSED_TARGETS 10 // 最大记录通过目标数量

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

// 新增：临时减速状态和已通过目标记录
static Temp_Slow_State slow_state = {false, 0};
static Passed_Target passed_targets[MAX_PASSED_TARGETS];
static uint32 passed_target_count = 0;
static int32 current_base_speed = -20; // 当前基础速度

// 新增：智能搜索状态
static Smart_Search_State smart_search = {false, 0, 0, {0}, false};

// #pragma section all "cpu1_dsram"
static uint8 binary_front[IMG_WIDTH * IMG_HEIGHT];
static uint8 binary_rear[IMG_WIDTH * IMG_HEIGHT];
// #pragma section all restore

// 函数声明 - 新增减速相关函数
static void handle_single_camera(void);
static void handle_dual_camera(void);
static void handle_target_logic(void);
static void execute_state_control(void);
static bool is_similar_target(const Component_Info *target1, const Component_Info *target2);
static void update_tracking(void);
static Component_Info *find_best_target(void);
static Pass_Strategy calc_pass_strategy(void);
static int32 get_offset_with_strategy(const Component_Info *target, Pass_Strategy strategy);
static void check_passed_targets(void);
static bool is_target_already_passed(const Component_Info *target);
static void record_passed_target(const Component_Info *target);
static void start_temp_slow(void);
static int32 calculate_current_speed(void);
static void update_smart_search(void);
static void start_smart_search(void);
static int32 get_smart_search_direction(void);

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

    // 初始化临时减速状态
    slow_state.is_active = false;
    slow_state.remaining_frames = 0;

    // 初始化已通过目标记录
    passed_target_count = 0;
    for (uint32 i = 0; i < MAX_PASSED_TARGETS; i++)
    {
        passed_targets[i].is_recorded = false;
    }

    // 初始化智能搜索状态
    smart_search.is_active = false;
    smart_search.remaining_frames = 0;
    smart_search.search_direction = 0;
    smart_search.has_last_second_target = false;
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

    // 检查是否有目标通过，并记录减速
    check_passed_targets();

    // 检查目标并处理转向逻辑
    handle_target_logic();
    execute_state_control();

    lcd_show_int(0, 7, curr_state, 1);
    lcd_show_int(3, 7, passed_target_count, 1); // 显示已通过目标数量
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
        // 使用智能目标选择
        Component_Info *best_target = find_best_target();
        if (best_target && best_target->center.x > 0 && best_target->center.y > 0 &&
            best_target->center.x != -1 && best_target->center.y != -1)
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

        // 启动智能搜索（如果有上次的第二大目标记录）
        if (smart_search.has_last_second_target && !smart_search.is_active && no_target_counter > 10)
        {
            start_smart_search();
        }

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
        smart_search.is_active = false;

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
    // 更新智能搜索状态
    update_smart_search();

    // 计算通过策略
    Pass_Strategy strategy = calc_pass_strategy();

    // 根据通过目标数量动态计算当前速度
    int32 dynamic_speed = calculate_current_speed();

    switch (curr_state)
    {
    case STATE_TURNING:
        guide_set_target_vel(dynamic_speed);
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
        // 转向对准后加速前进追踪目标，使用动态速度
        guide_set_target_vel(dynamic_speed);
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
        guide_set_target_vel(dynamic_speed);
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
        // 搜索状态：如果正在智能搜索，使用智能搜索方向
        guide_set_target_vel(SEARCHING_SPEED);

        if (smart_search.is_active)
        {
            // 使用智能搜索方向
            int32 smart_direction = get_smart_search_direction();
            guide_set_target_turn(smart_direction);
        }
        else
        {
            // 使用默认搜索方向
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
        // 寻找最佳有效目标
        Component_Info *best_target = find_best_target();
        if (is_target_valid(best_target))
        {
            has_valid_detection = true;
            current_detection = best_target;
        }
    }

    if (has_valid_detection)
    {
        if (tracked_target.is_valid)
        {
            // 目标跟踪中，检查目标是否仍然有效
            if (is_similar_target(&tracked_target.target, &results[0]))
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
            if (results[0].bbox.area > 0)
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

static Component_Info *find_best_target(void)
{
    Component_Info *best_target = NULL;
    uint32 best_area = 0;

    for (uint32 i = 0; i < component_count; i++)
    {
        // 只考虑有效的目标
        if (is_target_valid(&results[i]) && results[i].bbox.area > best_area)
        {
            best_area = results[i].bbox.area;
            best_target = &results[i];
        }
    }

    return best_target;
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

    // 记录第二大目标信息用于智能搜索
    smart_search.last_second_target = *second_target;
    smart_search.has_last_second_target = true;

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
        return target->bbox.min_x - IMG_WIDTH / 2;

    case PASS_STRATEGY_RIGHT:
        // 从右侧通过：目标位置向右偏移
        return target->bbox.max_x - IMG_WIDTH / 2;

    case PASS_STRATEGY_CENTER:
    default:
        // 直接瞄准目标中心去灭灯
        return target->center.x - IMG_WIDTH / 2;
    }
}

static void check_passed_targets(void)
{
    // 更新临时减速状态
    if (slow_state.is_active)
    {
        slow_state.remaining_frames--;
        if (slow_state.remaining_frames == 0)
        {
            slow_state.is_active = false; // 减速时间结束，恢复正常速度
        }
    }

    // 遍历当前检测到的所有目标
    for (uint32 i = 0; i < component_count; i++)
    {
        Component_Info *current_target = &results[i];

        // 检查目标是否足够大，表示车辆接近或通过了该目标
        if (current_target->bbox.area >= PASSED_TARGET_AREA_THRESHOLD || current_target->center.y > 108)
        {
            // 检查这个目标是否已经被记录过
            if (!is_target_already_passed(current_target))
            {
                // 记录新的通过目标并启动临时减速
                record_passed_target(current_target);
                start_temp_slow(); // 启动临时减速
            }
        }
    }
}

static bool is_target_already_passed(const Component_Info *target)
{
    // 检查目标是否已经被记录为通过目标
    for (uint32 i = 0; i < passed_target_count; i++)
    {
        Passed_Target *passed_target = &passed_targets[i];
        // 使用位置距离判断是否是同一个目标
        if (abs(passed_target->position.x - target->center.x) < 30 &&
            abs(passed_target->position.y - target->center.y) < 30)
        {
            return true;
        }
    }
    return false;
}

static void record_passed_target(const Component_Info *target)
{
    // 记录已通过目标
    if (passed_target_count < MAX_PASSED_TARGETS)
    {
        Passed_Target *new_passed_target = &passed_targets[passed_target_count];
        new_passed_target->position = target->center;
        new_passed_target->area = target->bbox.area;
        new_passed_target->is_recorded = true;
        passed_target_count++;
    }
}

static void start_temp_slow(void)
{
    // 启动临时减速
    slow_state.is_active = true;
    slow_state.remaining_frames = SLOW_DURATION_FRAMES;
}

static int32 calculate_current_speed(void)
{
    // 如果正在临时减速，返回减速速度
    if (slow_state.is_active)
    {
        return TEMP_SLOW_SPEED;
    }

    // 否则返回正常速度
    switch (curr_state)
    {
    case STATE_TURNING:
        return TURNING_SPEED;
    case STATE_TRACKING:
        return TRACKING_SPEED;
    case STATE_CLOSE:
        return CLOSE_SPEED;
    case STATE_SEARCHING:
    default:
        return SEARCHING_SPEED;
    }
}

static void update_smart_search(void)
{
    if (smart_search.is_active)
    {
        smart_search.remaining_frames--;

        // 智能搜索逻辑：根据上次的第二大目标调整搜索方向
        if (smart_search.has_last_second_target)
        {
            int32 direction = get_smart_search_direction();
            guide_set_target_turn(direction);
        }

        // 搜索持续时间到达后，停止智能搜索
        if (smart_search.remaining_frames == 0)
        {
            smart_search.is_active = false;
        }
    }
}

static void start_smart_search(void)
{
    smart_search.is_active = true;
    smart_search.remaining_frames = SMART_SEARCH_DURATION;
    smart_search.search_direction = (ROLL >= 0) ? SMART_SEARCH_TURN_SPEED : -SMART_SEARCH_TURN_SPEED;
}

static int32 get_smart_search_direction(void)
{
    // 根据上次的第二大目标位置和当前车辆位置，计算转向方向
    if (smart_search.has_last_second_target)
    {
        Component_Info *target = &smart_search.last_second_target;
        int32 direction = target->center.x - IMG_WIDTH / 2;

        // 添加偏移量以避免直接朝向目标
        direction += (direction > 0) ? -SIDE_PASS_OFFSET : SIDE_PASS_OFFSET;

        return direction;
    }

    // 默认搜索方向
    return (ROLL >= 0) ? SMART_SEARCH_TURN_SPEED : -SMART_SEARCH_TURN_SPEED;
}
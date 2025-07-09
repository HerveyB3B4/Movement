#include "guide.h"
#include "image.h"
#include "control.h"
#include "distance.h"
#include "detection.h"
#include "pid.h"
#include "receiver.h"

static int32 guide_target_vel = 0;
static int32 guide_target_turn = 0;

static int16 search_turn_value = 1500; // 目标丢失时的转向强度

// --- 静态（私有）函数声明 ---
static void handle_searching(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image);
static void handle_approaching(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image);
static void handle_circling(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image);
static void find_global_best_target(uint8 *front_image, uint8 *rear_image, connected_component_info *out_target, active_camera_t *out_cam);

/**
 * @brief 初始化状态机。
 */
void state_machine_init(state_machine_context_t *sm)
{
    if (sm == NULL)
        return;

    sm->current_state = STATE_SEARCHING;
    sm->current_target.is_valid = false;
    sm->active_camera = CAM_NONE;
    sm->state_timer = 0;
}

/**
 * @brief 运行状态机的主要函数。
 */
void state_machine_run(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image)
{
    if (sm == NULL)
        return;

    // 根据当前状态调用相应的处理函数
    switch (sm->current_state)
    {
    case STATE_SEARCHING:
        handle_searching(sm, front_image, rear_image);
        break;
    case STATE_APPROACHING_TARGET:
        handle_approaching(sm, front_image, rear_image);
        break;
    case STATE_CIRCLING_TARGET:
        handle_circling(sm, front_image, rear_image);
        break;
    default:
        // 如果状态未知，重置状态机
        state_machine_init(sm);
        break;
    }
    sm->state_timer++; // 每次迭代增加计时器
}

/**
 * @brief 获取当前状态。
 */
state_t state_machine_get_current_state(const state_machine_context_t *sm)
{
    return sm->current_state;
}

/**
 * @brief 获取当前活动的摄像头。
 */
active_camera_t state_machine_get_active_camera(const state_machine_context_t *sm)
{
    return sm->active_camera;
}

/**
 * @brief 从前后两个摄像头中寻找全局最优目标。
 */
static void find_global_best_target(uint8 *front_image, uint8 *rear_image, connected_component_info *out_target, active_camera_t *out_cam)
{
    static connected_component_info sorted_front[MAX_REGIONS];
    static connected_component_info sorted_rear[MAX_REGIONS];

    connected_component_info front_best = {.is_valid = false};
    connected_component_info rear_best = {.is_valid = false};

    // 1. 处理前置摄像头
    if (front_image != NULL)
    {
        uint8 count = find_and_sort_components_by_proximity(front_image, ALGORITHM_TWO_PASS, sorted_front, MAX_REGIONS);
        if (count > 0)
        {
            front_best = sorted_front[0];
        }
    }

    // 2. 处理后置摄像头
    if (rear_image != NULL)
    {
        uint8 count = find_and_sort_components_by_proximity(rear_image, ALGORITHM_TWO_PASS, sorted_rear, MAX_REGIONS);
        if (count > 0)
        {
            rear_best = sorted_rear[0];
        }
    }

    // 3. 决策：比较两个摄像头找到的最佳目标
    if (front_best.is_valid && !rear_best.is_valid)
    {
        *out_target = front_best;
        *out_cam = CAM_FRONT;
    }
    else if (!front_best.is_valid && rear_best.is_valid)
    {
        *out_target = rear_best;
        *out_cam = CAM_REAR;
    }
    else if (front_best.is_valid && rear_best.is_valid)
    {
        // 两个摄像头都找到了目标，比较哪个更优
        // 优先级1：比较面积，面积大的优先
        if (front_best.area > rear_best.area)
        {
            *out_target = front_best;
            *out_cam = CAM_FRONT;
        }
        else if (rear_best.area > front_best.area)
        {
            *out_target = rear_best;
            *out_cam = CAM_REAR;
        }
        else
        {
            // 优先级2：面积相等，比较与中心的距离，距离近的优先
            const int16 center_x = IMG_WIDTH / 2;
            const int16 center_y = IMG_HEIGHT / 2;
            int32 dx_f = front_best.center.x - center_x;
            int32 dy_f = front_best.center.y - center_y;
            uint32 dist_sq_f = dx_f * dx_f + dy_f * dy_f;
            int32 dx_r = rear_best.center.x - center_x;
            int32 dy_r = rear_best.center.y - center_y;
            uint32 dist_sq_r = dx_r * dx_r + dy_r * dy_r;

            // 距离近的优先，如果距离也相等，则优先前置摄像头
            if (dist_sq_f <= dist_sq_r)
            {
                *out_target = front_best;
                *out_cam = CAM_FRONT;
            }
            else
            {
                *out_target = rear_best;
                *out_cam = CAM_REAR;
            }
        }
    }
    else
    {
        // 两个摄像头都没找到目标
        out_target->is_valid = false;
        *out_cam = CAM_NONE;
    }
}

/**
 * @brief “寻找目标”状态的处理逻辑。
 */
static void handle_searching(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image)
{
    find_global_best_target(front_image, rear_image, &sm->current_target, &sm->active_camera);

    if (sm->current_target.is_valid)
    {
        // 找到了目标，切换到“接近目标”状态
        sm->current_state = STATE_APPROACHING_TARGET;
        sm->state_timer = 0; // 重置计时器
        // TODO: 修改运动逻辑
        // guide_set_target_turn(0); // 停止原地旋转
    }
    else
    {
        // TODO: 修改运动逻辑
        // 未找到目标，原地旋转寻找
        // guide_set_target_vel(0);
        // guide_set_target_turn(search_turn_value);
    }
}

/**
 * @brief “接近目标”状态的处理逻辑。
 */
static void handle_approaching(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image)
{
    find_global_best_target(front_image, rear_image, &sm->current_target, &sm->active_camera);

    if (sm->current_target.is_valid)
    {
        sm->state_timer = 0; // 只要能看到目标就重置丢失计时器

        // 计算到目标中心的距离
        int32_t dx = sm->current_target.center.x - (IMG_WIDTH / 2);
        int32_t dy = sm->current_target.center.y - (IMG_HEIGHT / 2);
        uint32_t distance_sq = dx * dx + dy * dy;

        // 检查是否足够近
        if (distance_sq < (APPROACH_DISTANCE_THRESHOLD * APPROACH_DISTANCE_THRESHOLD))
        {
            // 切换到“环绕目标”状态
            sm->current_state = STATE_CIRCLING_TARGET;
            sm->state_timer = 0;
            // TODO: 修改运动逻辑
            // guide_set_target_vel(0); // 到达后先停下
        }
        else
        {
            // TODO: 修改运动逻辑
            // // 还不够近，继续接近
            // guide_set_target_vel(GUIDE_CONSTANT_VEL);
            // // 根据目标x坐标偏差调整转向
            // int16 turn_error = sm->current_target.center.x - (IMG_WIDTH / 2);
            // // 如果是后置摄像头，转向应该反向
            // if (sm->active_camera == CAM_REAR)
            // {
            //     turn_error = -turn_error;
            // }
            // guide_set_target_turn(turn_error); // 此处可替换为PID控制器输出
        }
    }
    else
    {
        // 目标丢失，检查超时
        if (sm->state_timer > TARGET_LOST_TIMEOUT)
        {
            sm->current_target.is_valid = false;
            sm->active_camera = CAM_NONE;
            sm->current_state = STATE_SEARCHING;
            sm->state_timer = 0;
        }
    }
}

/**
 * @brief “环绕目标”状态的处理逻辑。
 */
static void handle_circling(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image)
{
    find_global_best_target(front_image, rear_image, &sm->current_target, &sm->active_camera);

    if (sm->current_target.is_valid)
    {
        sm->state_timer = 0; // 重置目标丢失计时器

        // TODO: 修改运动逻辑
        // // 实现环绕逻辑：设置一个固定的转向值，并用PID保持与目标的距离
        // guide_set_target_turn(1000); // 设置一个固定的转向值

        // // 使用PID控制器根据距离调整前进/后退速度以保持环绕半径
        // // 此处简化为固定慢速前进
        // guide_set_target_vel(-500);
    }
    else
    {
        // 目标丢失，检查超时
        if (sm->state_timer > TARGET_LOST_TIMEOUT)
        {
            sm->current_target.is_valid = false;
            sm->active_camera = CAM_NONE;
            sm->current_state = STATE_SEARCHING;
            sm->state_timer = 0;
        }
    }
}

void guide_set_target_vel(int32 target_vel)
{
    restrictValueI(&target_vel, -9999, 9999);
    guide_target_vel = -target_vel;
}

void guide_set_target_turn(int32 target_turn)
{
    // TODO 限幅
    guide_target_turn = target_turn;
    // AI:
    // // 对转向值进行PID计算或直接使用
    // // 此处可以加入PID控制器来平滑转向
    // guide_target_turn = PID_calc(&turn_PID, (float)target_turn);
    // restrictValueI(&guide_target_turn, -5000, 5000); // 限制转向输出
}

int16 get_guide_target_vel(void)
{
    return guide_target_vel;
}

int16 get_guide_target_turn(void)
{
    return guide_target_turn;
}

void guide_receiver(struct Control_Target *control_target)
{
    // 直接更新目标
    control_target->bottom_vel = (float)guide_target_vel;
    control_target->turn_err = g_turn_error;
    // AI:
    // control_target->bottom_vel = (float)guide_target_vel;
    // g_turn_error 应该由 guide_set_target_turn 更新
}

void guide_position_pid(struct Control_Target *control_target, Point *target)
{
    control_target->bottom_vel = PID_calc_Position(&bottom_position_PID,
                                                   distance_reckon(target->x, target->y, 0),
                                                   10.0f);
}

void guide_to_target(struct Control_Target *control_target)
{
    control_target->bottom_vel = (float)guide_target_vel;
    // control_target->turn_err = get_img_target_error();
    // control_target->bottom_vel = (float)guide_target_vel;
    // control_target->turn_err = get_img_target_error();
    // Point target_point = get_target_point();
    // guide_position_pid(control_target, &target_point);
    // AI:
    // guide_to_target 函数现在的作用就是将状态机计算出的速度和转向值应用到控制目标上
    // control_target->bottom_vel = (float)get_guide_target_vel();
    // control_target->turn_err = (float)get_guide_target_turn();
}

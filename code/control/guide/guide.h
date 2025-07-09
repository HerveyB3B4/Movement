#ifndef _CONTROL_GUIDE_H
#define _CONTROL_GUIDE_H

#include "zf_common_headfile.h"
#include "detection.h"

#define GUIDE_CONSTANT_VEL -2000 // 接近目标时的恒定速度
#define APPROACH_DISTANCE_THRESHOLD 25 // 接近目标的距离阈值 (像素)
#define TARGET_LOST_TIMEOUT 100        // 目标丢失的超时计数

// 定义状态机的三种状态
typedef enum {
    STATE_SEARCHING,          // 状态：寻找目标
    STATE_APPROACHING_TARGET, // 状态：接近目标
    STATE_CIRCLING_TARGET     // 状态：环绕目标
} state_t;

// 定义当前活动摄像头
typedef enum {
    CAM_NONE,  // 无有效目标
    CAM_FRONT, // 前置摄像头
    CAM_REAR   // 后置摄像头
} active_camera_t;


// 状态机上下文结构
// 用于存储状态机的当前状态和相关数据
typedef struct {
    state_t current_state;                  // 当前状态
    connected_component_info current_target; // 当前追踪的目标信息
    active_camera_t active_camera;          // 哪个摄像头正在追踪目标
    uint32_t state_timer;                   // 一个简单的计时器，可用于状态内的超时逻辑
} state_machine_context_t;


/**
 * @brief 初始化状态机。
 * @param sm 指向状态机上下文的指针。
 */
void state_machine_init(state_machine_context_t *sm);

/**
 * @brief 运行状态机的一个迭代。
 * @param sm 指向状态机上下文的指针。
 * @param front_image 指向前置摄像头的二值化图像数据。
 * @param rear_image 指向后置摄像头的二值化图像数据。
 */
void state_machine_run(state_machine_context_t *sm, uint8 *front_image, uint8 *rear_image);

/**
 * @brief 获取状态机的当前状态。
 * @param sm 指向状态机上下文的指针。
 * @return 当前状态 (state_t)。
 */
state_t state_machine_get_current_state(const state_machine_context_t *sm);

/**
 * @brief 获取当前活动的摄像头。
 * @param sm 指向状态机上下文的指针。
 * @return 当前活动的摄像头 (active_camera_t)。
 */
active_camera_t state_machine_get_active_camera(const state_machine_context_t *sm);

// 前向声明，避免循环包含
struct Control_Target;

int16 get_guide_target_vel(void);
int16 get_guide_target_turn(void);

void guide_set_target_vel(int32 target_vel);
void guide_set_target_turn(int32 target_turn);

void guide_receiver(struct Control_Target *control_target);
void guide_to_target(struct Control_Target *control_target);

#endif

#include "state_machine.h"
#include "detection.h"

static Run_State curr_state;
static Camera_Mode curr_camera_mode = SINGLE_CAMERA;
static uint8 front_img = 0;
static Component_Info *results = NULL;
static uint32 component_count = 0;

#pragma section all "cpu1_dsram"
static uint8 binary_front[IMG_WIDTH * IMG_HEIGHT];
static uint8 binary_rear[IMG_WIDTH * IMG_HEIGHT];
#pragma section all restore

void state_machine_init(Camera_Mode mode)
{
    curr_state = STATE_SEARCHING;
    curr_camera_mode = mode;
    detection_init(ALGORITHM_TWO_PASS, NULL);
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
}

static void state_machine_single_handler()
{
    if (mt9v03x_finish_flag)
    {
        binary_otsu(mt9v03x_image, binary_front);
        component_count = detection_find_components(binary_front, 0); // 前摄像头
        results = detection_get_results();
        // 排序
        qsort(results, component_count, sizeof(Component_Info), compare_components);

        // reset status
        mt9v03x_finish_flag = 0;
    }
}

static void state_machine_dual_handler()
{
    if (mt9v03x_finish_flag)
    {
        // 前摄像头
        binary_otsu(mt9v03x_image, binary_front);
        component_count = detection_find_components(binary_front, 0);
        results = detection_get_results();
        qsort(results, component_count, sizeof(Component_Info), compare_components);

        // 后摄像头
        binary_otsu(mt9v03x2_image, binary_rear);
        uint16 rear_count = detection_find_components(binary_rear, 1);
        Component_Info *rear_results = detection_get_results();
        qsort(rear_results, rear_count, sizeof(Component_Info), compare_components);

        // 比较两个摄像头获取到的结果

        // reset status
        mt9v03x_finish_flag = 0;
    }
}
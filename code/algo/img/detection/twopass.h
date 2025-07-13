#ifndef _DETECTION_TWOPASS_H
#define _DETECTION_TWOPASS_H

#include "detection.h"

typedef struct
{
    uint16 parent;      // 父节点
    Component_Box bbox; // 边界框信息
} DSU_Node;

// 函数声明
uint16 find_components_two_pass(uint8 *binary_image, uint8 camera_id, Component_Info *output);

#endif
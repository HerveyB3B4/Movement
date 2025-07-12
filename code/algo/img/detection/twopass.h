#ifndef _DETECTION_TWOPASS_H
#define _DETECTION_TWOPASS_H

#include "zf_common_headfile.h"

typedef struct
{
    uint16 parent;      // 父节点
    Component_Box bbox; // 边界框信息
} DSU_Node;

// 函数声明
uint16 find_components_two_pass(uint8 *binary_image);
Component_Info *get_twopass_res(void);

#endif
/**
 ******************************************************************************
 * @file    ekf platform.h
 * @author  xiang yankai  项彦凯
 * @version V1.2.2
 * @date    2024/2/28
 * @brief   当时尝试的 没想到成功了
 ******************************************************************************
 * @attention
 * 如果智能车感觉好用 欢迎支持支持我
 ******************************************************************************
 */
#ifndef __EKF_Platform_H
#define __EKF_Platform_H

//============================================================================
#include "zf_common_headfile.h"    //逐飞科技的库 给智能车用的
#include "Ifx_LutLSincosF32.h"    //英飞凌illd的库
#include "Ifx_LutAtan2F32.h"       //英飞凌illd的库
#include "matrix.h"   //arm平台的不需要引用 直接使用dsp库的 我自己实现的矩阵库
//这个是针对不同平台的三角函数 下面的是英飞凌的 其他大家自己补充
//=============================================================================


//==========================================================================
//英飞凌的illd的cos优化 如果不想要优化 直接cos和atan2  arm的dsp库自带这些 arm不需要
#define arm_cos_f32 Ifx_LutLSincosF32_cos  //英飞凌的cos
#define arm_atan2_f32 Ifx_LutAtan2F32_float32 //英飞凌的反atan2

//====================================================================================

//=====================================================================
//下面是矩阵库，如果是arm内核的就可以不用引用matrix库，arm内核需要注释掉下面 因为dsp里面有，这个是我自己实现的方便移植到各个单片机
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32
//==================================================================================================================
#endif //__EKF_Platform_H

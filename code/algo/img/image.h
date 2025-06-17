#ifndef _IMAGE_H
#define _IMAGE_H
#include "zf_common_headfile.h"

#define IMG_WIDTH MT9V03X_W  // 图像宽度
#define IMG_HEIGHT MT9V03X_H // 图像高度

// 在图像上画十字
// 参数：
// - img: 图像数据
// - center: 十字中心点
// - size: 十字大小(臂长)
// - color: 十字颜色(0x00为黑色，0xFF为白色)
void draw_cross(uint8_t *img, Point center, uint8_t size, uint8_t color);
void draw_middleline(uint8_t *img, uint8_t color);

int16 get_img_target_error();
int16 get_img_target_distance(); // 其实现在就是y轴

#endif
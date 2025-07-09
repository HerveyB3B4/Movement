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
void draw_cross(uint8 *img, Point center, uint8 size, uint8 color);
void draw_Vmiddleline(uint8 *img, uint8 color);
void draw_Hmiddleline(uint8 *img, uint8 color);
void draw_Vline(uint8 *img, uint8 x, uint8 color);
void draw_Hline(uint8 *img, uint8 y, uint8 color);
void draw_rectangle(uint8 *img, uint16 x, uint16 y, uint16 width, uint16 height, uint8 color);

int16 get_img_target_error();

void img_handler_alltarget();
void img_handler(uint8 lcd_flag);
Point get_target_point();

#endif
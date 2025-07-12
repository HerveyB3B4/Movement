#ifndef _DETECTION_FLOODFILL_H
#define _DETECTION_FLOODFILL_H

#include "zf_common_headfile.h"

uint16 find_components_flood_fill(uint8 *binary_image);
Component_Info *get_floodfill_res(void);

#endif
#ifndef _DETECTION_FLOODFILL_H
#define _DETECTION_FLOODFILL_H

#include "detection.h"

uint16 find_components_flood_fill(uint8 *binary_image, uint8 camera_id, Component_Info *output);

#endif
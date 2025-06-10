#ifndef _IMG_BINARY_H
#define _IMG_BINARY_H

#include "zf_common_headfile.h"

void binary_otsu(uint8 *input, uint8 *output);
void binary_otsu_improved(uint8 *input, uint8 *output);
void binary_otsu_2d(uint8 *input, uint8 *output);

#endif
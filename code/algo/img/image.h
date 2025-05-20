#ifndef _IMAGE_H
#define _IMAGE_H
#include "zf_common_headfile.h"

float otsuThreshold(const uint8* image, uint16 width, uint16 height);
void binarizeImage(const uint8* image,
                   uint8* binaryImage,
                   uint16 width,
                   uint16 height,
                   double threshold);
uint16 find(uint16* parent, uint16 x);
void unionSets(uint16* parent, uint16* rank, uint16 x, uint16 y);
void twoPassLabeling(const uint8_t* binaryImage,
                     uint16 width,
                     uint16 height,
                     uint16* labeledImage);
uint16 findLargestBlobLabel(uint16* labeledImage,
                            uint16 width,
                            uint16 height,
                            uint16* maxArea);
void computeCentroid(uint16* labeledImage,
                     uint16 width,
                     uint16 height,
                     uint16 label,
                     uint16* out_x,
                     uint16* out_y);
void drawRedPlus(uint8* image,
                 uint16 width,
                 uint16 height,
                 uint16 cx,
                 uint16 cy,
                 uint16 size);

#endif
#ifndef LOCATION_H
#define LOCATION_H

#include "common.h"

void location_init(void);
void location_update(float yaw, int encoder);
void location_get_coordinate(PointF *coordinate);

#endif // LOCATION_H
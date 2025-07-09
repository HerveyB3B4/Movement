#ifndef _COORDINATE_H_
#define _COORDINATE_H_

#include "zf_common_headfile.h"

PointF car_to_world(Point body_point, Point car_position, float car_yaw);
PointF world_to_car(PointF world_point, Point car_position, float car_yaw);

#endif
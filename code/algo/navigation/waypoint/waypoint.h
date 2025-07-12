#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "common.h"

#define TARGET_NUM 50

void waypoint_clear(void);
void waypoint_set(PointF *point);
void waypoint_get(int index, PointF *point);
float waypoint_claculate_distance(PointF *point1, PointF *point2);
float waypoint_claculate_azimuth_angle(PointF *point1, PointF *point2);

#endif
#include "location.h"

static PointF s_current_coordinate;

void location_init(void)
{
    s_current_coordinate.x = 0;
    s_current_coordinate.y = 0;
}

void location_update(float yaw, float encoder)
{
    s_current_coordinate.x += cos(yaw) * encoder;
    s_current_coordinate.y += sin(yaw) * encoder;
}

void location_get_coordinate(PointF *coordinate)
{
    *coordinate = s_current_coordinate;
}
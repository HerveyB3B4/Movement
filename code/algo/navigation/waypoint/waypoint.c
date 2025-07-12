#include "waypoint.h"

#include "imu.h"
#include "define.h"

static uint8 s_waypoint_count;
static PointF s_waypoint_coordinate[TARGET_NUM];

void waypoint_clear(void)
{
    s_waypoint_count = TARGET_NUM;
    while (s_waypoint_count--)
    {
        s_waypoint_coordinate[s_waypoint_count].x = 0;
        s_waypoint_coordinate[s_waypoint_count].y = 0;
    }
}

void waypoint_set(PointF *point)
{
    s_waypoint_coordinate[s_waypoint_count].x = point->x;
    s_waypoint_coordinate[s_waypoint_count].y = point->y;
    s_waypoint_count++;
}

void waypoint_get(int index, PointF *point)
{
    point->x = s_waypoint_coordinate[index].x;
    point->y = s_waypoint_coordinate[index].y;
}

float waypoint_claculate_distance(PointF *point1, PointF *point2)
{
    float distance = 0.0f;
    distance = sqrt((point1->x - point2->x) * (point1->x - point2->x) + (point1->y - point2->y) * (point1->y - point2->y));
    // TODO: 可能需要单位转换
    return distance;
}

static float my_abs(float num)
{
    return num < 0 ? -num : num;
}

float waypoint_claculate_azimuth_angle(PointF *point1, PointF *point2)
{
    float azimuth_angle = 0;
    if (point2->x != point1->x)
    {                                                                                    // The divisor cannot be 0
        azimuth_angle = atan(my_abs((point2->y - point1->y) / (point2->x - point1->x))); // the value must be positive
        azimuth_angle = azimuth_angle * DEG2RAD;
        if (point2->y - point1->y > 0 && point2->x - point1->x > 0) // first quadrant
            azimuth_angle = 90 - azimuth_angle;
        else if (point2->y - point1->y < 0 && point2->x - point1->x > 0) // four quadrant
            azimuth_angle = azimuth_angle + 90;
        else if (point2->y - point1->y < 0 && point2->x - point1->x < 0) // third quadrant
            azimuth_angle = 270 - azimuth_angle;
        else // second quadrant
            azimuth_angle = azimuth_angle + 270;
    }
    return azimuth_angle;
}

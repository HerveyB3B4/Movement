#include "coordinate.h"

// 将车身坐标系中的点转化到世界坐标系中
PointF car_to_world(Point body_point, Point car_position, float car_yaw)
{
    // yaw单位为度
    float radian = car_yaw * (3.14159265358979323846 / 180.0); // 将角度转换为弧度
    float cos_yaw = cos(radian);
    float sin_yaw = sin(radian);

    PointF world_point;
    world_point.x = car_position.x + body_point.x * cos_yaw - body_point.y * sin_yaw;
    world_point.y = car_position.y + body_point.x * sin_yaw + body_point.y * cos_yaw;
    return world_point;
}

// 将世界坐标系中的点转化到车身坐标系中
PointF world_to_car(PointF world_point, Point car_position, float car_yaw)
{
    float radian = car_yaw * (3.14159265358979323846 / 180.0); // 将角度转换为弧度
    float cos_yaw = cos(radian);
    float sin_yaw = sin(radian);

    PointF body_point;
    body_point.x = (world_point.x - car_position.x) * cos_yaw + (world_point.y - car_position.y) * sin_yaw;
    body_point.y = -(world_point.x - car_position.x) * sin_yaw + (world_point.y - car_position.y) * cos_yaw;
    return body_point;
}
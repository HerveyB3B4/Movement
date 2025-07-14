#include "distance.h"
#include "Attitude.h"
#include "image.h"

static float distance_model(int16 x, int16 y);

float distance_reckon(int16 x, int16 y, float c)
{
    // 直线距离估计
    if (x >= (IMG_WIDTH / 2) || y < 0 || y >= IMG_HEIGHT)
    {
        return 0;
    }

    float distance = distance_model(x, y);
    if (distance < 0)
    {
        return 0;
    }
    return distance + c;
}

float distance_reckon_horizontal(int16 x, int16 y, int16 c)
{
    // 横向偏移估计
    if (y <= 0)
        return 0;
    float distance = 1.7890 * x + 0.5660 * y + (-0.0010) * x * x + (-0.0012) * y * y + (-0.0126) * x * y + (-35.4742);
    return distance + c;
}

float distance_reckon_sinyaw(int16 x, int16 y, int16 c)
{
    // sin yaw估计，c为补偿项
    float sin_yaw = sqrt(1 - distance_reckon_horizontal(x, y, c) / distance_reckon(x, y, c));
    return sin_yaw;
}

int16 get_image_horizon()
{
    // 地平线
    float horizon = 1.756f * CAMERA_ANGLE + 1.391f * PITCH + (-90.160f);
    if (horizon < 0)
    {
        horizon = 0;
    }
    else if (horizon > IMG_HEIGHT - 1)
    {
        horizon = IMG_HEIGHT - 1;
    }

    return (int16)horizon;
}

static float distance_model(int16 x, int16 y)
{

    float ans = 0.0977 * x + (-7.8046) * y + 0.0043 * x * x + 0.0371 * y * y + (-0.0010) * x * y + 431.4145;

    return ans;
}

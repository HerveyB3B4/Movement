#include "distance.h"
#include "Attitude.h"
#include "image.h"

static float distance_w(int x);
static int16 distance_h(int16 y);

float distance_reckon(int16 x, int16 y)
{
    return (sqrtf((float)(x * x + y * y))) - 0;
}

static int16 distance_h(int16 y)
{
    return y;
}

static float distance_w(int x)
{
    return x;
}

int16 get_image_horizon()
{
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
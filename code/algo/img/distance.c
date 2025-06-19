#include "distance.h"

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
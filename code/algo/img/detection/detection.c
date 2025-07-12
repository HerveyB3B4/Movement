#include "detection.h"
#include "image.h"
#include "stdlib.h"
#include "twopass.h"
#include "floodfill.h"

static Component_AlgorithmEnum current_algorithm = ALGORITHM_TWO_PASS;
static uint16 component_count = 0;
static Component_Info *res = NULL;
static Component_Info tmp[MAX_REGIONS];

void detection_init(Component_AlgorithmEnum algo, Component_Info *output)
{
    current_algorithm = algo;
    init_union_find();
    if (output != NULL)
    {
        res = output;
    }
    else
    {
        res = tmp;
    }
}
void detection_find_components(uint8 *binary_image)
{
    if (binary_image == NULL)
        return;

    switch (current_algorithm)
    {
    case ALGORITHM_FLOOD_FILL:
        component_count = find_components_flood_fill(binary_image);
        res = get_floodfill_res();
        break;

    case ALGORITHM_TWO_PASS:
    default:
        component_count = find_components_two_pass(binary_image);
        res = get_twopass_res();
        break;
    }
}

Component_Info *detection_get_results(void)
{
    return res;
}
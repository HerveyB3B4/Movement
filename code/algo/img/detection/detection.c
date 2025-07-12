#include "detection.h"
#include "image.h"
#include "stdlib.h"
#include "twopass.h"
#include "floodfill.h"

static Component_AlgorithmEnum current_algorithm = ALGORITHM_TWO_PASS;

void detection_init(Component_AlgorithmEnum algo)
{
    current_algorithm = algo;
}

uint16 detection_find_components(uint8 *binary_image, uint8 camera_id, Component_Info *output)
{
    if (binary_image == NULL)
        return 0;
    switch (current_algorithm)
    {
    case ALGORITHM_FLOOD_FILL:
        return find_components_flood_fill(binary_image, camera_id, output);
        break;

    case ALGORITHM_TWO_PASS:
    default:
        return find_components_two_pass(binary_image, camera_id, output);
        break;
    }
}
#include "detection.h"
#include "image.h"
#include "stdlib.h"

// 并查集数组，用于两遍扫描算法
static uint16 parent[MAX_REGIONS];
static uint32 area[MAX_REGIONS];
static uint16 component_min_x[MAX_REGIONS];
static uint16 component_max_x[MAX_REGIONS];
static uint16 component_min_y[MAX_REGIONS];
static uint16 component_max_y[MAX_REGIONS];
static uint16 label_count = 0;

// 全局存储所有连通域信息
static connected_component_info all_components[MAX_REGIONS];
static uint8 total_found_components = 0;

// 临时标签数组
static uint16 temp_labels[IMG_HEIGHT][IMG_WIDTH];
static component_analysis_result find_largest_connected_component_with_algo(
    uint8 *binary_image,
    connected_component_algorithm_enum algorithm);

// 种子填充用的区域信息
typedef struct
{
    int16 min_x, min_y, max_x, max_y;
    uint32 area;
} flood_fill_region;

// 静态辅助函数声明
static component_analysis_result find_components_flood_fill(uint8 *binary_image);
static component_analysis_result find_components_two_pass(uint8 *binary_image);
static void flood_fill(uint8 *binary, uint16 x, uint16 y, uint8 label, flood_fill_region *region);
static uint8 find_all_centers_flood_fill(uint8 *binary_image, Point *points_array, uint8 max_points);
static uint8 find_all_centers_two_pass(uint8 *binary_image, Point *points_array, uint8 max_points);
static void init_union_find();

// 初始化并查集
static void init_union_find()
{
    for (uint16 i = 0; i < MAX_REGIONS; i++)
    {
        parent[i] = i;
        area[i] = 0;
        component_min_x[i] = IMG_WIDTH;
        component_max_x[i] = 0;
        component_min_y[i] = IMG_HEIGHT;
        component_max_y[i] = 0;
    }
    label_count = 1;
}

// 查找根节点（路径压缩）
uint16 find_root(uint16 x)
{
    if (parent[x] != x)
    {
        parent[x] = find_root(parent[x]);
    }
    return parent[x];
}

// 合并两个集合
void union_sets(uint16 x, uint16 y)
{
    uint16 root_x = find_root(x);
    uint16 root_y = find_root(y);

    if (root_x != root_y)
    {
        parent[root_y] = root_x;
        area[root_x] += area[root_y];

        // 更新边界框
        if (component_min_x[root_y] < component_min_x[root_x])
            component_min_x[root_x] = component_min_x[root_y];
        if (component_max_x[root_y] > component_max_x[root_x])
            component_max_x[root_x] = component_max_x[root_y];
        if (component_min_y[root_y] < component_min_y[root_x])
            component_min_y[root_x] = component_min_y[root_y];
        if (component_max_y[root_y] > component_max_y[root_x])
            component_max_y[root_x] = component_max_y[root_y];

        area[root_y] = 0;
    }
}

// 种子填充算法
static void flood_fill(uint8 *binary, uint16 x, uint16 y, uint8 label, flood_fill_region *region)
{
    // 使用栈替代递归
    static int16 stack_x[STACK_SIZE];
    static int16 stack_y[STACK_SIZE];
    int16 stack_pos = 0;

    stack_x[stack_pos] = x;
    stack_y[stack_pos] = y;
    stack_pos++;

    while (stack_pos > 0)
    {
        stack_pos--;
        x = stack_x[stack_pos];
        y = stack_y[stack_pos];

        if (x < 0 || x >= IMG_WIDTH || y < 0 || y >= IMG_HEIGHT ||
            binary[y * IMG_WIDTH + x] != 0xFF)
        {
            continue;
        }

        binary[y * IMG_WIDTH + x] = label;

        // 更新区域信息
        if (x < region->min_x)
            region->min_x = x;
        if (x > region->max_x)
            region->max_x = x;
        if (y < region->min_y)
            region->min_y = y;
        if (y > region->max_y)
            region->max_y = y;
        region->area++;

        // 四邻域扩展
        if (stack_pos < STACK_SIZE - 4)
        {
            stack_x[stack_pos] = x;
            stack_y[stack_pos] = y - 1;
            stack_pos++;
            stack_x[stack_pos] = x + 1;
            stack_y[stack_pos] = y;
            stack_pos++;
            stack_x[stack_pos] = x;
            stack_y[stack_pos] = y + 1;
            stack_pos++;
            stack_x[stack_pos] = x - 1;
            stack_y[stack_pos] = y;
            stack_pos++;
        }
    }
}

// 重构后的flood fill算法 - 一次性收集所有连通域
static component_analysis_result find_components_flood_fill(uint8 *binary_image)
{
    component_analysis_result result = {0};
    result.largest_component.is_valid = false;
    total_found_components = 0;

    static flood_fill_region regions[MAX_REGIONS];
    static uint8 temp_img[IMG_WIDTH * IMG_HEIGHT];
    uint8 region_count = 0;
    uint32 max_area = 0;
    uint8 max_idx = 0;

    memcpy(temp_img, binary_image, IMG_WIDTH * IMG_HEIGHT);

    // 一次扫描收集所有连通域
    for (uint16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (uint16 x = 0; x < IMG_WIDTH; x++)
        {
            if (temp_img[y * IMG_WIDTH + x] == 0xFF)
            {
                result.total_white_pixels++;

                if (region_count >= MAX_REGIONS)
                    break;

                // 初始化新区域
                regions[region_count].min_x = x;
                regions[region_count].min_y = y;
                regions[region_count].max_x = x;
                regions[region_count].max_y = y;
                regions[region_count].area = 0;

                flood_fill(temp_img, x, y, region_count + 1, &regions[region_count]);

                // 检查是否为有效连通域
                connected_component_info temp_component = {
                    .area = regions[region_count].area,
                    .min_x = regions[region_count].min_x,
                    .max_x = regions[region_count].max_x,
                    .min_y = regions[region_count].min_y,
                    .max_y = regions[region_count].max_y};

                if (is_valid_target(&temp_component))
                {
                    // 存储到全局数组
                    all_components[total_found_components] = temp_component;
                    all_components[total_found_components].center.x = (temp_component.min_x + temp_component.max_x) / 2;
                    all_components[total_found_components].center.y = (temp_component.min_y + temp_component.max_y) / 2;
                    all_components[total_found_components].is_valid = true;

                    // 更新最大连通域
                    if (temp_component.area > max_area)
                    {
                        max_area = temp_component.area;
                        max_idx = total_found_components;
                    }

                    total_found_components++;
                }
                region_count++;
            }
        }
    }

    result.total_components = total_found_components;

    // 设置最大连通域信息
    if (total_found_components > 0)
    {
        result.largest_component = all_components[max_idx];
    }

    return result;
}

// 重构后的two-pass算法 - 一次性收集所有连通域
static component_analysis_result find_components_two_pass(uint8 *binary_image)
{
    component_analysis_result result = {0};
    result.largest_component.is_valid = false;
    total_found_components = 0;

    init_union_find();

    // 清空标签数组
    for (int16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (int16 x = 0; x < IMG_WIDTH; x++)
        {
            temp_labels[y][x] = 0;
        }
    }

    // 第一趟扫描
    for (int16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (int16 x = 0; x < IMG_WIDTH; x++)
        {
            if (binary_image[y * IMG_WIDTH + x] == 0xFF)
            {
                result.total_white_pixels++;

                uint16 left_label = (x > 0) ? temp_labels[y][x - 1] : 0;
                uint16 up_label = (y > 0) ? temp_labels[y - 1][x] : 0;

                if (left_label == 0 && up_label == 0)
                {
                    if (label_count < MAX_REGIONS)
                    {
                        temp_labels[y][x] = label_count;
                        area[label_count] = 1;
                        component_min_x[label_count] = x;
                        component_max_x[label_count] = x;
                        component_min_y[label_count] = y;
                        component_max_y[label_count] = y;
                        label_count++;
                    }
                }
                else if (left_label != 0 && up_label == 0)
                {
                    temp_labels[y][x] = left_label;
                    uint16 root = find_root(left_label);
                    area[root]++;
                    if (x > component_max_x[root])
                        component_max_x[root] = x;
                    if (y > component_max_y[root])
                        component_max_y[root] = y;
                }
                else if (left_label == 0 && up_label != 0)
                {
                    temp_labels[y][x] = up_label;
                    uint16 root = find_root(up_label);
                    area[root]++;
                    if (x > component_max_x[root])
                        component_max_x[root] = x;
                    if (y > component_max_y[root])
                        component_max_y[root] = y;
                }
                else
                {
                    temp_labels[y][x] = left_label;
                    uint16 root = find_root(left_label);
                    area[root]++;
                    if (x > component_max_x[root])
                        component_max_x[root] = x;
                    if (y > component_max_y[root])
                        component_max_y[root] = y;

                    if (left_label != up_label)
                    {
                        union_sets(left_label, up_label);
                    }
                }
            }
        }
    }

    // 收集所有有效连通域信息
    static bool processed[MAX_REGIONS] = {false};
    uint32 max_area = 0;
    uint8 max_idx = 0;

    // 重置处理标记
    for (uint16 i = 0; i < MAX_REGIONS; i++)
    {
        processed[i] = false;
    }

    for (uint16 i = 1; i < label_count; i++)
    {
        uint16 root = find_root(i);
        if (area[root] > 0 && !processed[root])
        {
            processed[root] = true;

            connected_component_info temp_component = {
                .area = area[root],
                .min_x = component_min_x[root],
                .max_x = component_max_x[root],
                .min_y = component_min_y[root],
                .max_y = component_max_y[root]};

            if (is_valid_target(&temp_component))
            {
                // 计算精确中心点
                int32 sum_x = 0, sum_y = 0;
                uint32 count = 0;

                for (int16 y = 0; y < IMG_HEIGHT; y++)
                {
                    for (int16 x = 0; x < IMG_WIDTH; x++)
                    {
                        if (temp_labels[y][x] != 0 && find_root(temp_labels[y][x]) == root)
                        {
                            sum_x += x;
                            sum_y += y;
                            count++;
                        }
                    }
                }

                if (count > 0)
                {
                    // 存储到全局数组
                    all_components[total_found_components] = temp_component;
                    all_components[total_found_components].center.x = sum_x / count;
                    all_components[total_found_components].center.y = sum_y / count;
                    all_components[total_found_components].is_valid = true;

                    // 更新最大连通域
                    if (temp_component.area > max_area)
                    {
                        max_area = temp_component.area;
                        max_idx = total_found_components;
                    }

                    total_found_components++;
                }
            }
        }
    }

    result.total_components = total_found_components;

    // 设置最大连通域信息
    if (total_found_components > 0)
    {
        result.largest_component = all_components[max_idx];
    }

    return result;
}

// 简化的获取所有目标点坐标接口
uint8 find_all_white_centers(uint8 *binary_image, Point *points_array, uint8 max_points, connected_component_algorithm_enum algorithm)
{
    if (binary_image == NULL || points_array == NULL || max_points == 0)
    {
        return 0;
    }

    // 执行连通域分析
    find_largest_connected_component_with_algo(binary_image, algorithm);

    // 复制结果到用户数组
    uint8 copy_count = (total_found_components < max_points) ? total_found_components : max_points;
    for (uint8 i = 0; i < copy_count; i++)
    {
        points_array[i] = all_components[i].center;
    }

    return copy_count;
}

uint8 get_all_components_info(connected_component_info *components_array, uint8 max_components)
{
    if (components_array == NULL || max_components == 0)
    {
        return 0;
    }

    uint8 copy_count = (total_found_components < max_components) ? total_found_components : max_components;
    for (uint8 i = 0; i < copy_count; i++)
    {
        components_array[i] = all_components[i];
    }

    return copy_count;
}

uint8 get_total_components_count(void)
{
    return total_found_components;
}

Point find_white_center(uint8 *binary_image, connected_component_algorithm_enum algorithm)
{
    component_analysis_result result = find_largest_connected_component_with_algo(binary_image, algorithm);

    // 如果没有找到有效的连通域，返回(-1, -1)表示无效
    if (!result.largest_component.is_valid)
    {
        Point invalid_point = {-1, -1};
        return invalid_point;
    }

    return result.largest_component.center;
}

static component_analysis_result find_largest_connected_component_with_algo(
    uint8 *binary_image,
    connected_component_algorithm_enum algorithm)
{
    component_analysis_result result = {0};

    if (binary_image == NULL)
        return result;

    switch (algorithm)
    {
    case ALGORITHM_FLOOD_FILL:
        return find_components_flood_fill(binary_image);

    case ALGORITHM_TWO_PASS:
    default:
        return find_components_two_pass(binary_image);
    }
}

uint8 find_and_sort_components_by_proximity(
    uint8 *binary_image,
    connected_component_algorithm_enum algorithm,
    connected_component_info *sorted_components_array,
    uint8 max_components)
{
    if (binary_image == NULL || sorted_components_array == NULL || max_components == 0)
    {
        return 0;
    }

    find_largest_connected_component_with_algo(binary_image, algorithm);

    if (total_found_components == 0)
    {
        return 0;
    }

    qsort(all_components, total_found_components, sizeof(connected_component_info), compare_components);

    uint8 copy_count = (total_found_components < max_components) ? total_found_components : max_components;
    for (uint8 i = 0; i < copy_count; i++)
    {
        sorted_components_array[i] = all_components[i];
    }

    return copy_count;
}
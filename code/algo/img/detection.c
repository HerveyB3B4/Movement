#include "detection.h"
#include "image.h"
#include "stdlib.h"

// --- 模块内部数据结构 ---

// 并查集(DSU)节点信息
typedef struct
{
    uint16 parent;
    component_bbox_t bbox;
} dsu_node_t;

static uint16 label_count = 0;
// 全局存储所有连通域信息
static connected_component_info all_components[MAX_REGIONS];
static uint8 total_found_components = 0;

// 临时标签数组
static uint16 temp_labels[IMG_HEIGHT][IMG_WIDTH];
// 检测配置
static detection_config_t g_detection_config;

// 并查集数组，用于两遍扫描算法
static dsu_node_t dsu_sets[MAX_REGIONS];

// 静态辅助函数声明
static void analyze_components(uint8 *binary_image, connected_component_algorithm_enum algorithm);
static void find_components_flood_fill(uint8 *binary_image);
static void find_components_two_pass(uint8 *binary_image);
static void flood_fill(uint8 *binary, uint16 x, uint16 y, uint8 label, component_bbox_t *region);
static void init_union_find(void);
static uint16 find_root(uint16 x);
static void union_sets(uint16 x, uint16 y);

// --- 模块初始化 ---

void detection_init(detection_config_t *config)
{
    if (config != NULL)
    {
        g_detection_config = *config;
    }
    else
    {
        // 提供一个默认配置
        g_detection_config.algorithm = ALGORITHM_TWO_PASS;
        g_detection_config.sorted_components_array = NULL;
        g_detection_config.max_components = 0;
    }

    init_union_find();
    total_found_components = 0;
    memset(all_components, 0, sizeof(all_components));
}

// 初始化并查集
static void init_union_find(void)
{
    for (uint16 i = 0; i < MAX_REGIONS; i++)
    {
        dsu_sets[i].parent = i;
        dsu_sets[i].bbox.area = 0;
        dsu_sets[i].bbox.min_x = IMG_WIDTH;
        dsu_sets[i].bbox.max_x = 0;
        dsu_sets[i].bbox.min_y = IMG_HEIGHT;
        dsu_sets[i].bbox.max_y = 0;
    }
    label_count = 1;
}

// 查找根节点（路径压缩）
static uint16 find_root(uint16 i)
{
    uint16 root = i;
    while (dsu_sets[root].parent != root)
    {
        root = dsu_sets[root].parent;
    }
    while (dsu_sets[i].parent != root) {
        uint16 next = dsu_sets[i].parent;
        dsu_sets[i].parent = root;
        i = next;
    }
    return root;
}

// 合并两个集合
static void union_sets(uint16 x, uint16 y)
{
    uint16 root_x = find_root(x);
    uint16 root_y = find_root(y);

    if (root_x != root_y)
    {
        dsu_sets[root_y].parent = root_x;
        dsu_sets[root_x].bbox.area += dsu_sets[root_y].bbox.area;

        // 更新边界框
        if (dsu_sets[root_y].bbox.min_x < dsu_sets[root_x].bbox.min_x)
            dsu_sets[root_x].bbox.min_x = dsu_sets[root_y].bbox.min_x;
        if (dsu_sets[root_y].bbox.max_x > dsu_sets[root_x].bbox.max_x)
            dsu_sets[root_x].bbox.max_x = dsu_sets[root_y].bbox.max_x;
        if (dsu_sets[root_y].bbox.min_y < dsu_sets[root_x].bbox.min_y)
            dsu_sets[root_x].bbox.min_y = dsu_sets[root_y].bbox.min_y;
        if (dsu_sets[root_y].bbox.max_y > dsu_sets[root_x].bbox.max_y)
            dsu_sets[root_x].bbox.max_y = dsu_sets[root_y].bbox.max_y;

        dsu_sets[root_y].bbox.area = 0;
    }
}

// 种子填充算法
static void flood_fill(uint8 *binary, uint16 x, uint16 y, uint8 label, component_bbox_t *region)
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
static void find_components_flood_fill(uint8 *binary_image)
{
    total_found_components = 0;

    static component_bbox_t regions[MAX_REGIONS];
    static uint8 temp_img[IMG_WIDTH * IMG_HEIGHT];
    uint8 region_count = 0;

    memcpy(temp_img, binary_image, IMG_WIDTH * IMG_HEIGHT);

    // 一次扫描收集所有连通域
    for (uint16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (uint16 x = 0; x < IMG_WIDTH; x++)
        {
            if (temp_img[y * IMG_WIDTH + x] == 0xFF)
            {
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
                connected_component_info temp_component = {.bbox = {
                                                               .area = regions[region_count].area,
                                                               .min_x = regions[region_count].min_x,
                                                               .max_x = regions[region_count].max_x,
                                                               .min_y = regions[region_count].min_y,
                                                               .max_y = regions[region_count].max_y,
                                                           }};

                if (is_valid_target(&temp_component))
                {
                    // 存储到全局数组
                    all_components[total_found_components] = temp_component;
                    all_components[total_found_components].center.x = (temp_component.bbox.min_x + temp_component.bbox.max_x) / 2;
                    all_components[total_found_components].center.y = (temp_component.bbox.min_y + temp_component.bbox.max_y) / 2;

                    total_found_components++;
                }
                region_count++;
            }
        }
    }
}

// 重构后的two-pass算法 - 一次性收集所有连通域
static void find_components_two_pass(uint8 *binary_image)
{
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
                uint16 left_label = (x > 0) ? temp_labels[y][x - 1] : 0;
                uint16 up_label = (y > 0) ? temp_labels[y - 1][x] : 0;

                if (left_label == 0 && up_label == 0)
                {
                    if (label_count < MAX_REGIONS)
                    {
                        temp_labels[y][x] = label_count;
                        dsu_sets[label_count].bbox.area = 1;
                        dsu_sets[label_count].bbox.min_x = x;
                        dsu_sets[label_count].bbox.max_x = x;
                        dsu_sets[label_count].bbox.min_y = y;
                        dsu_sets[label_count].bbox.max_y = y;
                        label_count++;
                    }
                }
                else if (left_label != 0 && up_label == 0)
                {
                    temp_labels[y][x] = left_label;
                    uint16 root = find_root(left_label);
                    dsu_sets[root].bbox.area++;
                    if (x > dsu_sets[root].bbox.max_x)
                        dsu_sets[root].bbox.max_x = x;
                    if (y > dsu_sets[root].bbox.max_y)
                        dsu_sets[root].bbox.max_y = y;
                }
                else if (left_label == 0 && up_label != 0)
                {
                    temp_labels[y][x] = up_label;
                    uint16 root = find_root(up_label);
                    dsu_sets[root].bbox.area++;
                    if (x > dsu_sets[root].bbox.max_x)
                        dsu_sets[root].bbox.max_x = x;
                    if (y > dsu_sets[root].bbox.max_y)
                        dsu_sets[root].bbox.max_y = y;
                }
                else
                {
                    temp_labels[y][x] = left_label;
                    uint16 root = find_root(left_label);
                    dsu_sets[root].bbox.area++;
                    if (x > dsu_sets[root].bbox.max_x)
                        dsu_sets[root].bbox.max_x = x;
                    if (y > dsu_sets[root].bbox.max_y)
                        dsu_sets[root].bbox.max_y = y;

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

    // 重置处理标记
    for (uint16 i = 0; i < MAX_REGIONS; i++)
    {
        processed[i] = false;
    }

    for (uint16 i = 1; i < label_count; i++)
    {
        uint16 root = find_root(i);
        if (dsu_sets[root].bbox.area > 0 && !processed[root])
        {
            processed[root] = true;

            connected_component_info temp_component = {.bbox = {
                                                               .area = dsu_sets[root].bbox.area,
                                                               .min_x = dsu_sets[root].bbox.min_x,
                                                               .max_x = dsu_sets[root].bbox.max_x,
                                                               .min_y = dsu_sets[root].bbox.min_y,
                                                               .max_y = dsu_sets[root].bbox.max_y,
                                                           }};

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

                    total_found_components++;
                }
            }
        }
    }
}

static void analyze_components(uint8 *binary_image, connected_component_algorithm_enum algorithm)
{
    if (binary_image == NULL)
        return;

    switch (algorithm)
    {
    case ALGORITHM_FLOOD_FILL:
        find_components_flood_fill(binary_image);
        break;

    case ALGORITHM_TWO_PASS:
    default:
        find_components_two_pass(binary_image);
        break;
    }
}

uint8 find_and_sort_components_by_proximity(uint8 *binary_image)
{
    if (binary_image == NULL || g_detection_config.sorted_components_array == NULL || g_detection_config.max_components == 0)
    {
        return 0;
    }

    analyze_components(binary_image, g_detection_config.algorithm);

    if (total_found_components == 0)
    {
        return 0;
    }

    qsort(all_components, total_found_components, sizeof(connected_component_info), compare_components);

    uint8 copy_count = (total_found_components < g_detection_config.max_components) ? total_found_components : g_detection_config.max_components;
    for (uint8 i = 0; i < copy_count; i++)
    {
        g_detection_config.sorted_components_array[i] = all_components[i];
    }

    return copy_count;
}
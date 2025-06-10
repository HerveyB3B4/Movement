#include "twopass.h"
#include "image.h"

// 并查集数组，用于标记连通域
static uint16 parent[MAX_REGIONS];
static uint32 area[MAX_REGIONS];            // 记录每个连通域的面积
static uint16 component_min_x[MAX_REGIONS]; // 连通域边界框
static uint16 component_max_x[MAX_REGIONS];
static uint16 component_min_y[MAX_REGIONS];
static uint16 component_max_y[MAX_REGIONS];
static uint16 label_count = 0;

// 临时标签数组，只在函数执行期间使用
static uint16 temp_labels[IMG_HEIGHT][IMG_WIDTH];

// 初始化配置
void init_component_config(connected_component_config *config)
{
    if (config == NULL)
        return;

    config->min_area_threshold = 10; // 最小10个像素
    config->max_regions_limit = MAX_REGIONS;
    config->overflow_strategy = STRATEGY_DYNAMIC_THRESHOLD;
    config->merge_distance = 5; // 5像素距离内可合并
}

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
    label_count = 1; // 从1开始，0表示背景
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

// 检查是否应该忽略小连通域
static bool should_ignore_small_component(uint16 current_area, uint32 min_threshold)
{
    return current_area < min_threshold;
}

// 检查两个连通域是否足够近可以合并
static bool components_nearby(uint16 label1, uint16 label2, uint8 distance_threshold)
{
    uint16 root1 = find_root(label1);
    uint16 root2 = find_root(label2);

    if (root1 == root2)
        return false;

    // 计算边界框之间的距离
    int16 dx = 0, dy = 0;

    if (component_max_x[root1] < component_min_x[root2])
    {
        dx = component_min_x[root2] - component_max_x[root1];
    }
    else if (component_max_x[root2] < component_min_x[root1])
    {
        dx = component_min_x[root1] - component_max_x[root2];
    }

    if (component_max_y[root1] < component_min_y[root2])
    {
        dy = component_min_y[root2] - component_max_y[root1];
    }
    else if (component_max_y[root2] < component_min_y[root1])
    {
        dy = component_min_y[root1] - component_max_y[root2];
    }

    return (dx + dy) <= distance_threshold;
}

// 动态调整面积阈值
static uint32 adjust_threshold_dynamically(uint16 current_label_count, uint32 original_threshold)
{
    if (current_label_count > MAX_REGIONS * 0.8)
    {
        return original_threshold * 2; // 提高阈值
    }
    else if (current_label_count > MAX_REGIONS * 0.6)
    {
        return original_threshold * 1.5;
    }
    return original_threshold;
}

// 高级连通域分析函数
component_analysis_result find_largest_connected_component_advanced(
    uint8 *binary_image,
    connected_component_config *config)
{
    component_analysis_result result = {0};
    result.largest_component.center.x = -1;
    result.largest_component.center.y = -1;

    if (binary_image == NULL || config == NULL)
    {
        return result;
    }

    init_union_find();

    // 清空临时标签数组
    for (int16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (int16 x = 0; x < IMG_WIDTH; x++)
        {
            temp_labels[y][x] = 0;
        }
    }

    uint32 dynamic_threshold = config->min_area_threshold;
    bool early_termination_triggered = false;

    // 第一趟扫描：标记连通域
    for (int16 y = 0; y < IMG_HEIGHT && !early_termination_triggered; y++)
    {
        for (int16 x = 0; x < IMG_WIDTH && !early_termination_triggered; x++)
        {
            if (binary_image[y * IMG_WIDTH + x] == 0xFF)
            { // 白色像素
                result.total_white_pixels++;

                // 检查左边和上边的邻居
                uint16 left_label = 0;
                uint16 up_label = 0;

                if (x > 0 && temp_labels[y][x - 1] != 0)
                {
                    left_label = temp_labels[y][x - 1];
                }

                if (y > 0 && temp_labels[y - 1][x] != 0)
                {
                    up_label = temp_labels[y - 1][x];
                }

                if (left_label == 0 && up_label == 0)
                {
                    // 新的连通域
                    if (label_count < config->max_regions_limit)
                    {
                        temp_labels[y][x] = label_count;
                        area[label_count] = 1;

                        // 初始化边界框
                        component_min_x[label_count] = x;
                        component_max_x[label_count] = x;
                        component_min_y[label_count] = y;
                        component_max_y[label_count] = y;

                        label_count++;
                    }
                    else
                    {
                        // 处理溢出情况
                        result.overflow_occurred = true;

                        switch (config->overflow_strategy)
                        {
                        case STRATEGY_EARLY_TERMINATION:
                            early_termination_triggered = true;
                            break;

                        case STRATEGY_DYNAMIC_THRESHOLD:
                            dynamic_threshold = adjust_threshold_dynamically(label_count, config->min_area_threshold);
                            break;

                        case STRATEGY_IGNORE_SMALL:
                        case STRATEGY_MERGE_NEARBY:
                        default:
                            // 暂时跳过这个像素
                            break;
                        }
                    }
                }
                else if (left_label != 0 && up_label == 0)
                {
                    // 继承左边的标签
                    temp_labels[y][x] = left_label;
                    uint16 root = find_root(left_label);
                    area[root]++;

                    // 更新边界框
                    if (x > component_max_x[root])
                        component_max_x[root] = x;
                    if (y > component_max_y[root])
                        component_max_y[root] = y;
                }
                else if (left_label == 0 && up_label != 0)
                {
                    // 继承上边的标签
                    temp_labels[y][x] = up_label;
                    uint16 root = find_root(up_label);
                    area[root]++;

                    // 更新边界框
                    if (x > component_max_x[root])
                        component_max_x[root] = x;
                    if (y > component_max_y[root])
                        component_max_y[root] = y;
                }
                else
                {
                    // 左边和上边都有标签
                    temp_labels[y][x] = left_label;
                    uint16 root = find_root(left_label);
                    area[root]++;

                    // 更新边界框
                    if (x > component_max_x[root])
                        component_max_x[root] = x;
                    if (y > component_max_y[root])
                        component_max_y[root] = y;

                    // 如果标签不同，需要合并
                    if (left_label != up_label)
                    {
                        if (config->overflow_strategy == STRATEGY_MERGE_NEARBY)
                        {
                            if (components_nearby(left_label, up_label, config->merge_distance))
                            {
                                union_sets(left_label, up_label);
                            }
                        }
                        else
                        {
                            union_sets(left_label, up_label);
                        }
                    }
                }
            }
        }
    }

    // 第二趟扫描：找最大连通域并应用过滤策略
    uint16 max_label = 0;
    uint32 max_area = 0;
    uint16 valid_components = 0;

    // 找到最大面积的连通域
    for (uint16 i = 1; i < label_count; i++)
    {
        uint16 root = find_root(i);
        if (area[root] > 0)
        {
            // 应用面积过滤
            if (config->overflow_strategy == STRATEGY_IGNORE_SMALL ||
                config->overflow_strategy == STRATEGY_DYNAMIC_THRESHOLD)
            {
                if (should_ignore_small_component(area[root], dynamic_threshold))
                {
                    area[root] = 0; // 标记为无效
                    continue;
                }
            }

            valid_components++;
            if (area[root] > max_area)
            {
                max_area = area[root];
                max_label = root;
            }
        }
    }

    result.processed_components = valid_components;
    result.total_components = label_count - 1;

    if (max_area == 0)
    {
        return result; // 没有找到有效的连通域
    }

    // 计算最大连通域的详细信息
    int32 sum_x = 0, sum_y = 0;
    uint32 count = 0;

    for (int16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (int16 x = 0; x < IMG_WIDTH; x++)
        {
            if (temp_labels[y][x] != 0 && find_root(temp_labels[y][x]) == max_label)
            {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    if (count > 0)
    {
        result.largest_component.center.x = sum_x / count;
        result.largest_component.center.y = sum_y / count;
        result.largest_component.area = max_area;
        result.largest_component.min_x = component_min_x[max_label];
        result.largest_component.max_x = component_max_x[max_label];
        result.largest_component.min_y = component_min_y[max_label];
        result.largest_component.max_y = component_max_y[max_label];
        result.largest_component.label = max_label;
    }

    return result;
}

// 保持原有的简单接口
Point find_largest_connected_component(uint8 *binary_image)
{
    connected_component_config config;
    init_component_config(&config);

    component_analysis_result result = find_largest_connected_component_advanced(binary_image, &config);
    return result.largest_component.center;
}

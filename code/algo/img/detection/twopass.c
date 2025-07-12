#include "twopass.h"
#include "detection.h"

static DSU_Node dsu_sets[MAX_REGIONS];
static Component_Info res[MAX_REGIONS];
static uint16 labels_map[IMG_HEIGHT][IMG_WIDTH];
static uint16 current_label = 1;

static void union_find_init(void);
static uint16 union_find_root(uint16 i);
static void union_sets(uint16 x, uint16 y);
static uint16 find_root(uint16 x);
static uint16 update_res(uint8 camera_id);

uint16 find_components_two_pass(uint8 *binary_image, uint8 camera_id)
{
    // 初始化
    union_find_init();
    current_label = 1;

    // 清空标签映射
    for (uint16 i = 0; i < IMG_HEIGHT; i++)
    {
        for (uint16 j = 0; j < IMG_WIDTH; j++)
        {
            labels_map[i][j] = 0;
        }
    }

    // 第一次扫描：分配标签并建立等价关系
    for (uint16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (uint16 x = 0; x < IMG_WIDTH; x++)
        {
            if (binary_image[y * IMG_WIDTH + x] == 255) // 前景像素
            {
                uint16 left_label = 0, top_label = 0;

                // 检查左邻居
                if (x > 0 && labels_map[y][x - 1] != 0)
                {
                    left_label = labels_map[y][x - 1];
                }

                // 检查上邻居
                if (y > 0 && labels_map[y - 1][x] != 0)
                {
                    top_label = labels_map[y - 1][x];
                }

                if (left_label == 0 && top_label == 0)
                {
                    // 新连通区域
                    labels_map[y][x] = current_label;
                    dsu_sets[current_label].bbox.min_x = x;
                    dsu_sets[current_label].bbox.max_x = x;
                    dsu_sets[current_label].bbox.min_y = y;
                    dsu_sets[current_label].bbox.max_y = y;
                    dsu_sets[current_label].bbox.area = 1;
                    current_label++;
                }
                else if (left_label != 0 && top_label == 0)
                {
                    // 使用左邻居的标签
                    labels_map[y][x] = left_label;
                    uint16 root = union_find_root(left_label);
                    dsu_sets[root].bbox.area++;
                    if (x > dsu_sets[root].bbox.max_x)
                        dsu_sets[root].bbox.max_x = x;
                    if (y > dsu_sets[root].bbox.max_y)
                        dsu_sets[root].bbox.max_y = y;
                    if (x < dsu_sets[root].bbox.min_x)
                        dsu_sets[root].bbox.min_x = x;
                    if (y < dsu_sets[root].bbox.min_y)
                        dsu_sets[root].bbox.min_y = y;
                }
                else if (left_label == 0 && top_label != 0)
                {
                    // 使用上邻居的标签
                    labels_map[y][x] = top_label;
                    uint16 root = union_find_root(top_label);
                    dsu_sets[root].bbox.area++;
                    if (x > dsu_sets[root].bbox.max_x)
                        dsu_sets[root].bbox.max_x = x;
                    if (y > dsu_sets[root].bbox.max_y)
                        dsu_sets[root].bbox.max_y = y;
                    if (x < dsu_sets[root].bbox.min_x)
                        dsu_sets[root].bbox.min_x = x;
                    if (y < dsu_sets[root].bbox.min_y)
                        dsu_sets[root].bbox.min_y = y;
                }
                else
                {
                    // 两个邻居都有标签
                    labels_map[y][x] = left_label;
                    if (left_label != top_label)
                    {
                        union_sets(left_label, top_label);
                    }
                    uint16 root = union_find_root(left_label);
                    dsu_sets[root].bbox.area++;
                    if (x > dsu_sets[root].bbox.max_x)
                        dsu_sets[root].bbox.max_x = x;
                    if (y > dsu_sets[root].bbox.max_y)
                        dsu_sets[root].bbox.max_y = y;
                    if (x < dsu_sets[root].bbox.min_x)
                        dsu_sets[root].bbox.min_x = x;
                    if (y < dsu_sets[root].bbox.min_y)
                        dsu_sets[root].bbox.min_y = y;
                }
            }
        }
    }

    // 第二次扫描：更新所有像素的标签为其根标签
    for (uint16 y = 0; y < IMG_HEIGHT; y++)
    {
        for (uint16 x = 0; x < IMG_WIDTH; x++)
        {
            if (labels_map[y][x] != 0)
            {
                labels_map[y][x] = union_find_root(labels_map[y][x]);
            }
        }
    }

    // 更新结果数组并返回连通域数量
    return update_res(camera_id);
}

static void union_find_init(void)
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
}

static uint16 union_find_root(uint16 i)
{
    if (dsu_sets[i].parent != i)
    {
        dsu_sets[i].parent = union_find_root(dsu_sets[i].parent); // 路径压缩
    }
    return dsu_sets[i].parent;
}

static uint16 find_root(uint16 x)
{
    return union_find_root(x);
}

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

static uint16 update_res(uint8 camera_id)
{
    uint16 component_count = 0;

    // 清空结果数组
    for (uint16 i = 0; i < MAX_REGIONS; i++)
    {
        res[i].center.x = 0;
        res[i].center.y = 0;
        res[i].bbox.area = 0;
        res[i].bbox.min_x = 0;
        res[i].bbox.max_x = 0;
        res[i].bbox.min_y = 0;
        res[i].bbox.max_y = 0;
    }

    // 遍历所有可能的根节点，找到有效的连通域
    for (uint16 i = 1; i < current_label; i++)
    {
        uint16 root = union_find_root(i);
        if (root == i && dsu_sets[i].bbox.area > 0)
        {
            // 这是一个有效的连通域根节点
            if (component_count < MAX_REGIONS)
            {
                res[component_count].bbox.area = dsu_sets[i].bbox.area;
                res[component_count].bbox.min_x = dsu_sets[i].bbox.min_x;
                res[component_count].bbox.max_x = dsu_sets[i].bbox.max_x;
                res[component_count].bbox.min_y = dsu_sets[i].bbox.min_y;
                res[component_count].bbox.max_y = dsu_sets[i].bbox.max_y;

                // 计算中心点
                res[component_count].center.x = (dsu_sets[i].bbox.min_x + dsu_sets[i].bbox.max_x) / 2;
                res[component_count].center.y = (dsu_sets[i].bbox.min_y + dsu_sets[i].bbox.max_y) / 2;
                res[component_count].camera_id = camera_id;

                component_count++;
            }
        }
    }

    return component_count;
}

Component_Info *get_twopass_res(void)
{
    return res;
}

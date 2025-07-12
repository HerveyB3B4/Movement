#include "floodfill.h"
#include "detection.h"

static Component_Info res[MAX_REGIONS];

static void flood_fill(uint8 *binary, uint16 x, uint16 y, uint8 label, Component_Box *region);
static uint16 update_res(Component_Box *regions, uint8 region_count);

uint16 find_components_flood_fill(uint8 *binary_image)
{
    static Component_Box regions[MAX_REGIONS];
    static uint8 temp_img[IMG_WIDTH * IMG_HEIGHT];
    uint8 region_count = 0;

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

    // 复制图像数据
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

                // 执行种子填充
                flood_fill(temp_img, x, y, region_count + 1, &regions[region_count]);

                region_count++;
            }
        }

        // 如果已经达到最大区域数，提前退出
        if (region_count >= MAX_REGIONS)
            break;
    }

    // 更新结果数组并返回连通域数量
    return update_res(regions, region_count);
}

static void flood_fill(uint8 *binary, uint16 x, uint16 y, uint8 label, Component_Box *region)
{
    // 使用栈替代递归，避免栈溢出
    static int16 stack_x[STACK_SIZE];
    static int16 stack_y[STACK_SIZE];
    int16 stack_pos = 0;

    // 初始化栈
    stack_x[stack_pos] = x;
    stack_y[stack_pos] = y;
    stack_pos++;

    while (stack_pos > 0)
    {
        // 弹出栈顶元素
        stack_pos--;
        x = stack_x[stack_pos];
        y = stack_y[stack_pos];

        // 边界检查和像素值检查
        if (x < 0 || x >= IMG_WIDTH || y < 0 || y >= IMG_HEIGHT ||
            binary[y * IMG_WIDTH + x] != 0xFF)
        {
            continue;
        }

        // 标记当前像素
        binary[y * IMG_WIDTH + x] = label;

        // 更新区域边界框信息
        if (x < region->min_x)
            region->min_x = x;
        if (x > region->max_x)
            region->max_x = x;
        if (y < region->min_y)
            region->min_y = y;
        if (y > region->max_y)
            region->max_y = y;
        region->area++;

        // 四邻域扩展（防止栈溢出）
        if (stack_pos < STACK_SIZE - 4)
        {
            // 上邻居
            stack_x[stack_pos] = x;
            stack_y[stack_pos] = y - 1;
            stack_pos++;

            // 右邻居
            stack_x[stack_pos] = x + 1;
            stack_y[stack_pos] = y;
            stack_pos++;

            // 下邻居
            stack_x[stack_pos] = x;
            stack_y[stack_pos] = y + 1;
            stack_pos++;

            // 左邻居
            stack_x[stack_pos] = x - 1;
            stack_y[stack_pos] = y;
            stack_pos++;
        }
    }
}

static uint16 update_res(Component_Box *regions, uint8 region_count)
{
    uint16 valid_component_count = 0;

    for (uint8 i = 0; i < region_count && i < MAX_REGIONS; i++)
    {
        if (regions[i].area > 0)
        {
            // 将 Component_Box 转换为 Component_Info
            res[valid_component_count].bbox.area = regions[i].area;
            res[valid_component_count].bbox.min_x = regions[i].min_x;
            res[valid_component_count].bbox.max_x = regions[i].max_x;
            res[valid_component_count].bbox.min_y = regions[i].min_y;
            res[valid_component_count].bbox.max_y = regions[i].max_y;

            // 计算中心点
            res[valid_component_count].center.x = (regions[i].min_x + regions[i].max_x) / 2;
            res[valid_component_count].center.y = (regions[i].min_y + regions[i].max_y) / 2;

            valid_component_count++;
        }
    }

    return valid_component_count;
}

Component_Info *get_floodfill_res(void)
{
    return res;
}

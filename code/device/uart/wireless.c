#include "wireless.h"

void wireless_init()
{
#ifdef USE_WIFI
    // wifi串口的连接方式
    // 我的建议是初始化用while包裹
    // while (wifi_uart_init(WIFI_SSID, WIFI_PASSWORD, WIFI_UART_STATION)) { // 初始化无线串口
    //     printf("wifi_uart_init failed\n");
    //     system_delay_ms(1000); // 等待1秒后重试
    // }
    // printf("wifi_uart_init success\n");
    // printf("connect mode  :%d\n", WIFI_UART_AUTO_CONNECT);                     // 模块工作模式
    // printf("module version:%s\n", wifi_uart_information.wifi_uart_version);    // 模块固件版本
    // printf("module mac    :%s\n", wifi_uart_information.wifi_uart_mac);        // 模块 MAC 信息
    // printf("module ip     :%s\n", wifi_uart_information.wifi_uart_local_ip);   // 模块 IP 地址
    // printf("module port   :%s\n", wifi_uart_information.wifi_uart_local_port); // 模块 PORT 信息
    if (wifi_uart_init(WIFI_SSID, WIFI_PASSWORD, WIFI_UART_STATION))
    { // 初始化无线串口
        printf("wifi_uart_init failed\n");
    }
    else
    {
        printf("wifi_uart_init success\n");
        printf("connect mode  :%d\n", WIFI_UART_AUTO_CONNECT);                     // 模块工作模式
        printf("module version:%s\n", wifi_uart_information.wifi_uart_version);    // 模块固件版本
        printf("module mac    :%s\n", wifi_uart_information.wifi_uart_mac);        // 模块 MAC 信息
        printf("module ip     :%s\n", wifi_uart_information.wifi_uart_local_ip);   // 模块 IP 地址
        printf("module port   :%s\n", wifi_uart_information.wifi_uart_local_port); // 模块 PORT 信息
    }
#else
    // 无线串口/无线蓝牙串口的连接方式
    if (wireless_uart_init())
    {
        printf("wireless_uart_init failed\n");
    }
    else
    {
        printf("wireless_uart_init success\n");
    }
#endif
}

void wireless_send_buffer(const uint8 *buff, uint16 len)
{
#ifdef USE_WIFI
    if (wifi_uart_send_buffer(buff, len) != len)
    {
        printf("wifi uart send buffer failed\n");
    }
    else
    {
        printf("wifi uart send buffer success, send_len: %u\n", len);
    }
#else
    if (wireless_uart_send_buffer(buff, len) != len)
    {
        printf("wifi uart send buffer failed\n");
    }
    else
    {
        printf("wifi uart send buffer success, send_len: %u\n", len);
    }
#endif
}

uint16 wireless_read_buffer(uint8 *buff, uint16 len)
{
#ifdef USE_WIFI
    uint16 data_length = (uint16)wifi_uart_read_buffer(buff, len);
#else
    uint16 data_length = (uint16)wireless_uart_read_buffer(buff, len);
#endif
    if (data_length)
    {
#ifdef USE_WIFI
        if (strstr((char *)buff, "+IPD")) // 判断数据格式是否是通过网络发送过来的数据
#endif
        {
            printf("wireless read buffer success, read_len: %u\n", data_length);
            printf("\r\n Get data: <%s>.", buff);
            memset(buff, 0, data_length);
        }
    }
    return data_length; // 返回实际读取数据长度
}
// 使用示例见 system/test/test.c
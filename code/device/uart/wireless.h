#ifndef _WIRELESS_H
#define _WIRELESS_H

#include "zf_common_headfile.h"

// 这些定义爱怎么改怎么改，我没有兴趣和你讨论代码规范
#define USE_WIFI           (1) // 定义是否使用 WiFi 模块 1-使用 0-不使用
#define WIFI_SSID          "B207_Super_Car"
#define WIFI_PASSWORD      "88888888"

void wireless_init(void);
void wireless_send_buffer(const uint8 *buff, uint16 len);
uint16 wireless_read_buffer(uint8 *buff, uint16 len);

#endif
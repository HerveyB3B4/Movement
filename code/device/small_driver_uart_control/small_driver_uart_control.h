#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"

#define USER_REV (0)

#define SMALL_DRIVER_UART_LEFT (UART_3)
#define SMALL_DRIVER_UART_RIGHT (UART_6)

#define SMALL_DRIVER_BAUDRATE (115200)

#define SMALL_DRIVER_RX_LEFT (UART3_TX_P20_0)
#define SMALL_DRIVER_RX_RIGHT (UART6_TX_P22_0)
#define SMALL_DRIVER_TX_LEFT (UART3_RX_P20_3)
#define SMALL_DRIVER_TX_RIGHT (UART6_RX_P23_1)

typedef struct {
    uint8 send_data_buffer[7];  // 发送缓冲数组

    uint8 receive_data_buffer[7];  // 接收缓冲数组

    uint8 receive_data_count;  // 接收计数

    uint8 sum_check_data;  // 校验位

    int16 receive_left_speed_data;  // 接收到的左侧电机速度数据

    int16 receive_right_speed_data;  // 接收到的右侧电机速度数据

} small_device_value_struct;

extern small_device_value_struct motor_value;

void uart_control_callback(uint8 uart_index);  // 无刷驱动 串口接收回调函数

void small_driver_set_duty(int16 left_duty,
                           int16 right_duty);  // 无刷驱动 设置电机占空比

void small_driver_get_speed(void);  // 无刷驱动 获取速度信息

void small_driver_uart_init(void);  // 无刷驱动 串口通讯初始化

#endif

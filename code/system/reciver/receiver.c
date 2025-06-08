#include "receiver.h"

void receiver_init()
{
    // TODO: uart init
    ble6a20_init();
}

/*
五位通信协议：
    0xA5, 0x00, 0xXX, 0xXX, 0xXX

    0xA5: 起始字节
    0x00: 指令字节
    0xXX: 高字节速度
    0xXX: 低字节速度
    0xXX: 校验和（前四个字节的和）
*/
uint8 receive_data_buffer[5];
uint8 receive_data_count = 0;
int16 g_received_vel = 0;

void receiver_callback()
{
    uint8 receive_data;
    if (uart_query_byte(RECIVER_UART_INDEX, &receive_data))
    {
        if (receive_data == 0xA5 && receive_data_buffer[0] != 0xA5)
        {
            receive_data_count = 0;
        }

        receive_data_buffer[receive_data_count++] =
            receive_data;

        if (receive_data_count >= 5)
        {
            if (receive_data_buffer[0] == 0xA5)
            {
                uint8 sum_check_data = 0;
                for (int i = 0; i < 4; i++)
                {
                    sum_check_data += receive_data_buffer[i];
                }

                if (sum_check_data == receive_data_buffer[4])
                {
                    g_received_vel = ((uint32)receive_data_buffer[2] << 8) |
                                     (uint32)receive_data_buffer[3];
                }
            }
            receive_data_count = 0;                                      // Reset count after processing
            memset(receive_data_buffer, 0, sizeof(receive_data_buffer)); // Clear buffer
        }
    }
}
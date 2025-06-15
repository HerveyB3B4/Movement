#include "receiver.h"
#include "control.h"
#include "pid.h"

void receiver_init()
{
    ble6a20_init();
}

uint8 receive_data_buffer[20];
uint8 receive_data_count = 0;
int16 g_received_vel = 0;
int16 g_turn_error = 0;

static float converter_bytes_to_float(uint8 *bytes)
{
    float_converter converter;
    converter.bytes = (uint32)bytes[3] << 24 |
                      (uint32)bytes[2] << 16 |
                      (uint32)bytes[1] << 8 |
                      (uint32)bytes[0];
    return converter.value;
}

static void process_pid_command(uint8 *data)
{
    uint8 checksum = 0;
    for (int j = 0; j < 14; j++)
    {
        checksum += data[j];
    }
    if (checksum == data[14])
    {
        float p = converter_bytes_to_float(&data[2]);
        float i = converter_bytes_to_float(&data[6]);
        float d = converter_bytes_to_float(&data[10]);
        float pid[3] = {p, i, d};
        switch (data[1])
        {
        // bottom
        case 0x05: // 角速度环
            PID_init_Position(&bottom_angle_velocity_PID, pid, 9999, 9999);
            break;

        case 0x06: // 角度环
            PID_init_Position(&bottom_angle_PID, pid, 9999, 9999);
            break;

        case 0x07: // 速度环
            PID_init_Position(&bottom_velocity_PID, pid, 9999, 10);
            break;

        // side
        case 0x08: // 角速度环
            PID_init_Position(&side_angle_velocity_PID, pid, 9999, 9999);
            break;
        case 0x09: // 角度环
            PID_init_Position(&side_angle_PID, pid, 9999, 9999);
            break;
        case 0x0A: // 速度环
            PID_init_Position(&side_velocity_PID, pid, 9999, 10);
            break;

        // turn
        case 0x0B: // 角速度环
            PID_init_Position(&turn_angle_velocity_PID, pid, 9999, 9999);
            break;
        case 0x0C: // 误差环
            PID_init_Position(&turn_error_PID, pid, 9999, 9999);
            break;
        case 0x0D: // 速度环
            PID_init_Position(&turn_velocity_PID, pid, 9999, 10);
            break;
        default:
            break;
        }
    }
}

/*
五位通信协议：
    0xA5, 0x00, 0xXX, 0xXX, 0xXX

    0xA5: 起始字节
    0x00: 指令字节
    0xXX: 高字节速度
    0xXX: 低字节速度
    0xXX: 校验和（前四个字节的和）

PID参数数据包格式（15字节）：
    0xA5, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX

    0xA5: 起始字节（固定）
    0xXX: 指令字节，用于区分不同的PID控制器：
          0x05 - 底轮角速度环PID
          0x06 - 底轮角度环PID
          0x07 - 底轮速度环PID
          0x08 - 侧轮角速度环PID
          0x09 - 侧轮角度环PID
          0x0A - 侧轮速度环PID
          0x0B - 转向角速度环PID
          0x0C - 转向误差环PID
          0x0D - 转向速度环PID
    0xXX, 0xXX, 0xXX, 0xXX: P参数（4字节浮点数，小端序）
    0xXX, 0xXX, 0xXX, 0xXX: I参数（4字节浮点数，小端序）
    0xXX, 0xXX, 0xXX, 0xXX: D参数（4字节浮点数，小端序）
    0xXX: 校验和（前14个字节的累加和）
*/
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
                    switch (receive_data_buffer[1])
                    {
                    case 0x00:
                        // 处理停止指令
                        g_received_vel = 0;
                        g_turn_error = 0;
                        break;
                    case 0x01:
                        // 处理速度指令
                        g_received_vel = ((int)receive_data_buffer[2] << 8) |
                                         (int)receive_data_buffer[3];
                        break;
                    case 0x02:
                        // 处理转向指令
                        g_turn_error = ((int)receive_data_buffer[2] << 8) |
                                       (int)receive_data_buffer[3];
                        break;
                    case 0x05:
                    case 0x06:
                    case 0x07:
                        // 处理底轮PID参数指令
                        process_pid_command(&receive_data_buffer);
                        break;
                    default:
                        // 未知指令，忽略
                        break;
                    }
                }
            }
            receive_data_count = 0;                                      // Reset count after processing
            memset(receive_data_buffer, 0, sizeof(receive_data_buffer)); // Clear buffer
        }
    }
}
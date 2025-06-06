#include "test.h"
#include "attitude.h"
#include "encoder.h"
#include "image.h"
#include "lcd.h"
#include "menu_input.h"
#include "motor.h"
#include "small_driver_uart_control.h"
#include "velocity.h"
#include "zf_common_headfile.h"
#include "wireless.h"
#include "sd_card.h" // 添加SD卡操作头文件

void test_bottom_motor()
{
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: forward");
    lcd_show_string(0, 1, "KEY_D: backward");
    lcd_show_string(0, 4, "Press KEY_L to exit");
    while (keymsg.key != KEY_L)
    {
        if (keymsg.key == KEY_U) // 向前
        {
            gpio_set_level(DIR_BOTTOM, 1);
            pwm_set_duty(MOTOR_BOTTOM, 10000);
        }
        if (keymsg.key == KEY_D) // 向后
        {
            gpio_set_level(DIR_BOTTOM, 0);
            pwm_set_duty(MOTOR_BOTTOM, 8000);
        }
        lcd_show_int(0, 5, g_vel_motor.bottom, 5);
        lcd_show_float(0, 6, g_vel_motor.bottomReal, 5, 5);
        lcd_show_float(0, 7, g_vel_motor.bottomFiltered, 5, 5);
        // encoder_clear_count(ENCODER_BOTTOM);
    }
    pwm_set_duty(MOTOR_BOTTOM, 0);
    lcd_clear();
}

void test_side_motor()
{
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: left  forward");
    lcd_show_string(0, 1, "KEY_D: left backward");
    lcd_show_string(0, 2, "KEY_R: right forward");
    lcd_show_string(0, 3, "KEY_L:right backward");
    lcd_show_string(0, 4, "Release to stop");
    lcd_show_string(0, 5, "Press KEY_B to exit");
    system_delay_ms(200); // 等待按键稳定
    while (keymsg.key != KEY_B)
    {
        // 根据按键状态设置电机速度
        if (keymsg.key == KEY_U) // 左电机向前
        {
            small_driver_set_duty(2000, 0);
        }
        else if (keymsg.key == KEY_D) // 左电机向后
        {
            small_driver_set_duty(-2000, 0);
        }
        else if (keymsg.key == KEY_R) // 右电机向前
        {
            small_driver_set_duty(0, 2000);
        }
        else if (keymsg.key == KEY_L) // 右电机向后
        {
            small_driver_set_duty(0, -2000);
        }
        else // 没有按键按下时停止电机
        {
            small_driver_set_duty(0, 0);
        }

        // 显示电机状态
        lcd_show_string(0, 6, "Front:");
        lcd_show_int(8, 6, g_vel_motor.momentumFront, 5);
        lcd_show_string(0, 7, "Back:");
        lcd_show_int(8, 7, g_vel_motor.momentumBack, 5);
    }

    // 退出前确保电机停止
    small_driver_set_duty(0, 0);
    lcd_clear();
}

// void test_side_motor() {
//     lcd_clear();
//     lcd_show_string(0, 0, "KEY_U: left  forward");
//     lcd_show_string(0, 1, "KEY_D: left backward");
//     lcd_show_string(0, 2, "KEY_B: right forward");
//     lcd_show_string(0, 3, "KEY_R:right backward");
//     lcd_show_string(0, 4, "Press KEY_L to exit");
//     int count = 0;
//     while (keymsg.key != KEY_L) {
//         if (keymsg.key == KEY_U) {
//             small_driver_set_duty(-600, 600);
//         } else if (keymsg.key == KEY_D) {
//             small_driver_set_duty(-5000, 5000);
//         }
//     }
//     small_driver_set_duty(0, 0);
//     lcd_clear();
// }

void test_attitude()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {
        lcd_show_string(0, 0, "Pitch:");
        lcd_show_float(0, 1, currentFrontAngle, 3, 3);
        lcd_show_float(8, 1, currentFrontAngle - g_menu_manual_param.mechanicalPitchAngle * 0.1f, 3, 3);
        lcd_show_string(0, 2, "Row:");
        lcd_show_float(0, 3, currentSideAngle, 3, 3);
        lcd_show_float(8, 3, currentSideAngle - g_menu_manual_param.mechanicalRollAngle * 0.1f, 3, 3);
        lcd_show_string(0, 4, "Yaw:");
        lcd_show_float(0, 5, yawAngle, 3, 3);
    }
    lcd_clear();
}

void test_imu()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {
        lcd_show_string(0, 0, "x:");
        lcd_show_float(0, 1, g_imu_data.gyro.x, 3, 3);
        lcd_show_float(8, 1, g_imu_data.acc.x, 3, 3);
        lcd_show_string(0, 2, "y:");
        lcd_show_float(0, 3, g_imu_data.gyro.y, 3, 3);
        lcd_show_float(8, 3, g_imu_data.acc.y, 3, 3);
        lcd_show_string(0, 4, "z:");
        lcd_show_float(0, 5, g_imu_data.gyro.z, 3, 3);
        lcd_show_float(8, 5, g_imu_data.acc.z, 3, 3);
    }
    lcd_clear();
}

void test_noise()
{
    lcd_clear();

    float sum_ax = 0.0f, sum_ax2 = 0.0f;
    float sum_ay = 0.0f, sum_ay2 = 0.0f;
    float sum_az = 0.0f, sum_az2 = 0.0f;
    float sum_gx = 0.0f, sum_gx2 = 0.0f;
    float sum_gy = 0.0f, sum_gy2 = 0.0f;
    float sum_gz = 0.0f, sum_gz2 = 0.0f;

    float var_ax = 0.0f;
    float var_ay = 0.0f;
    float var_az = 0.0f;
    float var_gx = 0.0f;
    float var_gy = 0.0f;
    float var_gz = 0.0f;

    uint8 T = 10;
    uint8 cnt = 0;
    uint32 total = 0;
    while (keymsg.key != KEY_L)
    {
        if (cnt < T)
        {
            // 读取传感器数据
            imu660rb_get_acc();
            imu660rb_get_gyro();

            // 实时计算并累加
            float current_ax =
                imu660rb_acc_transition(imu660rb_acc_x) * GravityAcc;
            float current_ay =
                imu660rb_acc_transition(imu660rb_acc_y) * GravityAcc;
            float current_az =
                imu660rb_acc_transition(imu660rb_acc_z) * GravityAcc;

            float current_gx =
                imu660rb_gyro_transition(imu660rb_gyro_x) * DEG2RAD;
            float current_gy =
                imu660rb_gyro_transition(imu660rb_gyro_y) * DEG2RAD;
            float current_gz =
                imu660rb_gyro_transition(imu660rb_gyro_z) * DEG2RAD;

            // 累加原始值和平方值
            sum_ax += current_ax;
            sum_ax2 += current_ax * current_ax;
            sum_ay += current_ay;
            sum_ay2 += current_ay * current_ay;
            sum_az += current_az;
            sum_az2 += current_az * current_az;

            sum_gx += current_gx;
            sum_gx2 += current_gx * current_gx;
            sum_gy += current_gy;
            sum_gy2 += current_gy * current_gy;
            sum_gz += current_gz;
            sum_gz2 += current_gz * current_gz;
        }
        else
        {
            // 计算均值
            float mean_ax = sum_ax / total;
            float mean_ay = sum_ay / total;
            float mean_az = sum_az / total;
            float mean_gx = sum_gx / total;
            float mean_gy = sum_gy / total;
            float mean_gz = sum_gz / total;

            var_ax = (sum_ax2 / total) - (mean_ax * mean_ax);
            var_ay = (sum_ay2 / total) - (mean_ay * mean_ay);
            var_az = (sum_az2 / total) - (mean_az * mean_az);
            var_gx = (sum_gx2 / total) - (mean_gx * mean_gx);
            var_gy = (sum_gy2 / total) - (mean_gy * mean_gy);
            var_gz = (sum_gz2 / total) - (mean_gz * mean_gz);

            // lcd_show_string(0, 0, "acc_x:");
            // lcd_show_float(8, 0, var_ax, 5, 3);
            // lcd_show_string(0, 1, "acc_y:");
            // lcd_show_float(8, 1, var_ay, 5, 3);
            // lcd_show_string(0, 2, "acc_z:");
            // lcd_show_float(8, 2, var_az, 5, 3);
            // lcd_show_string(0, 3, "gyro_x:");
            // lcd_show_float(8, 3, var_gx, 5, 3);
            // lcd_show_string(0, 4, "gyro_y:");
            // lcd_show_float(8, 4, var_gy, 5, 3);
            // lcd_show_string(0, 5, "gyro_z:");
            // lcd_show_float(8, 5, var_gz, 5, 3);
            printf(
                "total: %d, acc_x: %f, acc_y: %f, acc_z: %f, gyro_x: %f, "
                "gyro_y: %f, gyro_z: %f\n",
                total, var_ax, var_ay, var_az, var_gx, var_gy, var_gz);
            cnt = 0;
        }
        cnt++;
        total++;
        system_delay_ms(100);
    }
    lcd_clear();
}

void test_side_deadzone()
{
    lcd_clear();
    lcd_show_string(0, 0, "Auto testing...");
    lcd_show_string(0, 1, "Front PWM:");
    lcd_show_string(0, 2, "Front Speed:");
    lcd_show_string(0, 3, "Back PWM:");
    lcd_show_string(0, 4, "Back Speed:");
    lcd_show_string(0, 7, "Press KEY_L to exit");

    uint32 front_deadzone = 0;
    uint32 back_deadzone = 0;
    uint32 found_front = 0;
    uint32 found_back = 0;
    uint8 front_done = 0;
    uint8 back_done = 0;

    while (keymsg.key != KEY_L)
    {
        // 测试前电机
        if (!front_done)
        {
            front_deadzone += 1;
            set_momentum_motor_pwm(front_deadzone, back_deadzone);
            system_delay_ms(50);

            if (abs(g_vel_motor.momentumFront) > 0)
            {
                found_front = front_deadzone;
                front_done = 1;
            }
        }

        // 测试后电机
        if (!back_done)
        {
            back_deadzone += 1;
            set_momentum_motor_pwm(front_deadzone, back_deadzone);
            system_delay_ms(50);

            if (abs(g_vel_motor.momentumBack) > 0)
            {
                found_back = back_deadzone;
                back_done = 1;
            }
        }

        // 显示当前状态
        lcd_show_int(10, 1, front_deadzone, 5);
        lcd_show_int(10, 2, g_vel_motor.momentumFront, 5);
        lcd_show_int(10, 3, back_deadzone, 5);
        lcd_show_int(10, 4, g_vel_motor.momentumBack, 5);

        // 显示找到的死区值
        if (front_done)
        {
            lcd_show_string(0, 5, "Front min:");
            lcd_show_int(10, 5, found_front, 5);
        }
        if (back_done)
        {
            lcd_show_string(0, 6, "Back min:");
            lcd_show_int(10, 6, found_back, 5);
        }

        // 安全检查
        if (front_deadzone >= 10000 || back_deadzone >= 10000)
        {
            lcd_show_string(0, 7, "Error: Too high!");
            break;
        }

        // 如果两个都找到了就停止增加PWM
        if (front_done && back_done)
        {
            system_delay_ms(100); // 延时避免刷新太快
            small_driver_set_duty(0, 0);
        }
    }

    small_driver_set_duty(0, 0); // 停止电机
    lcd_clear();
}

void test_bottom_pwm()
{
    lcd_clear();
    lcd_show_string(0, 0, "KEY_U: PWM +100");
    lcd_show_string(0, 1, "KEY_D: PWM -100");
    lcd_show_string(0, 2, "KEY_R: Change dir");
    lcd_show_string(0, 3, "KEY_B: Set PWM");
    lcd_show_string(0, 4, "Press KEY_L to exit");

    int32 current_pwm = 0;
    int32 output_pwm = 0;
    uint8 direction = 1; // 1为正向，0为反向

    while (keymsg.key != KEY_L)
    {
        if (keymsg.key == KEY_U)
        {
            current_pwm += 10;
            if (current_pwm > 10000)
                current_pwm = 10000; // PWM上限
        }
        else if (keymsg.key == KEY_D)
        {
            current_pwm -= 10;
            if (current_pwm < 0)
                current_pwm = 0; // PWM下限
        }
        else if (keymsg.key == KEY_R)
        {
            direction = !direction;
            system_delay_ms(200); // 切换方向时增加延时防止抖动
        }
        else if (keymsg.key == KEY_B)
        {
            output_pwm = current_pwm; // 确认输出当前PWM值
            system_delay_ms(200);
        }

        // 更新电机输出
        gpio_set_level(DIR_BOTTOM, direction);
        pwm_set_duty(MOTOR_BOTTOM, output_pwm); // 使用确认后的PWM值

        // 显示当前状态
        lcd_show_string(0, 5, "Set PWM:");
        lcd_show_int(9, 5, current_pwm, 5);
        lcd_show_string(0, 6, "Out PWM:");
        lcd_show_int(9, 6, output_pwm, 5);
        lcd_show_string(0, 7, "Speed:");
        lcd_show_float(7, 7, g_vel_motor.bottomReal, 3, 2);

        system_delay_ms(50); // 调整延时控制PWM变化速度
    }

    pwm_set_duty(MOTOR_BOTTOM, 0); // 停止电机
    lcd_clear();
}

void test_bottom_deadzone()
{
    lcd_clear();
    lcd_show_string(0, 1, "Testing Forward...");
    lcd_show_string(0, 7, "Press KEY_L to exit");

    uint32 forward_deadzone = 0;
    uint32 backward_deadzone = 0;
    uint32 found_forward = 0;
    uint32 found_backward = 0;
    uint8 forward_done = 0;
    uint8 backward_done = 0;
    float speed_threshold = 0.1f; // 速度阈值，当检测到速度超过此值时认为电机已启动

    // 首先测试正向死区
    gpio_set_level(DIR_BOTTOM, 1); // 设置为正向

    while (keymsg.key != KEY_L)
    {
        // 测试正向死区
        if (!forward_done)
        {
            forward_deadzone += 5; // 每次增加5的PWM值
            pwm_set_duty(MOTOR_BOTTOM, forward_deadzone);
            system_delay_ms(100); // 给电机一些响应时间

            // 如果速度超过阈值，则确认找到死区值
            if (fabs(g_vel_motor.bottom) > speed_threshold)
            {
                found_forward = forward_deadzone;
                forward_done = 1;

                // 停止电机，准备测试反向
                pwm_set_duty(MOTOR_BOTTOM, 0);
                system_delay_ms(500);          // 等待电机完全停止
                gpio_set_level(DIR_BOTTOM, 0); // 设置为反向
            }
        }
        // 测试反向死区
        else if (!backward_done)
        {
            backward_deadzone += 5;
            pwm_set_duty(MOTOR_BOTTOM, backward_deadzone);
            system_delay_ms(100);

            if (fabs(g_vel_motor.bottom) > speed_threshold)
            {
                found_backward = backward_deadzone;
                backward_done = 1;

                // 停止电机
                pwm_set_duty(MOTOR_BOTTOM, 0);
            }
        }

        // 显示当前测试状态
        lcd_show_string(0, 2, "FPWM:");
        lcd_show_int(12, 2, forward_deadzone, 5);
        lcd_show_string(0, 3, "Speed:");
        lcd_show_float(7, 3, g_vel_motor.bottomReal, 3, 2);

        // 显示找到的死区值
        if (forward_done)
        {
            lcd_show_string(0, 4, "Fmin:");
            lcd_show_int(12, 4, found_forward, 5);
            lcd_show_string(0, 1, "Testing B");
        }

        if (backward_done)
        {
            lcd_show_string(0, 5, "Bmin:");
            lcd_show_int(12, 5, found_backward, 5);
            lcd_show_string(0, 1, "Complete");
        }

        // 如果两个方向都测试完成，就显示结果
        if (forward_done && backward_done)
        {
            lcd_show_string(0, 6, "Test done");
            system_delay_ms(100);
        }
    }

    // 退出前确保电机停止
    pwm_set_duty(MOTOR_BOTTOM, 0);
    lcd_clear();
}

void test_double_camera()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            // edge_detect_dynamic(mt9v03x_image, edge_map, &edge_cfg);
            tft180_show_gray_image(0, 0, mt9v03x_image, MT9V03X_W, MT9V03X_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
            tft180_show_gray_image(tft180_width_max - IMG_SIZE_W, 0,
                                   mt9v03x_image, MT9V03X_W, MT9V03X_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
        }
        if (mt9v03x2_finish_flag)
        {
            mt9v03x2_finish_flag = 0;
            // edge_detect_dynamic(mt9v03x2_image, edge_map2, &edge_cfg);
            tft180_show_gray_image(0, tft180_height_max - IMG_SIZE_H,
                                   mt9v03x2_image, MT9V03X_W, MT9V03X_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
            tft180_show_gray_image(tft180_width_max - IMG_SIZE_W,
                                   tft180_height_max - IMG_SIZE_H,
                                   mt9v03x2_image, MT9V03X2_W, MT9V03X2_H,
                                   IMG_SIZE_W, IMG_SIZE_H, 0);
        }
    }
    lcd_clear();
}

void test_image()
{
    lcd_clear();

    // 计算最佳缩放比例
    float scale_w = (float)tft180_width_max / MT9V03X_W;
    float scale_h = (float)tft180_height_max / MT9V03X_H;
    // 选择较小的缩放比例，保持宽高比
    float scale = scale_w < scale_h ? scale_w : scale_h;

    // 计算实际显示大小
    uint16 display_w = (uint16)(MT9V03X_W * scale);
    uint16 display_h = (uint16)(MT9V03X_H * scale);

    // 计算居中显示的起始坐标
    uint16 start_x = (tft180_width_max - display_w) / 2;
    uint16 start_y = (tft180_height_max - display_h) / 2;

    uint16_t edge_map[MT9V03X_W][MT9V03X_H];
    Point center = {-1, -1}; // 默认返回无效坐标
    while (keymsg.key != KEY_L)
    {
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            binary_otsu_improved(mt9v03x_image, edge_map);
            center = find_largest_white_region_center(edge_map);
            draw_cross(edge_map, center, -1, RGB565_YELLOW);
            tft180_show_gray_image(start_x, start_y, edge_map, MT9V03X_W,
                                   MT9V03X_H, display_w, display_h, 0);
        }
    }
    lcd_clear();
}

#define WIRELESS_BUFFER_SIZE 256 // 定义无线UART缓冲区大小
uint8 g_wireless_uart_buffer[WIRELESS_BUFFER_SIZE];

void test_wireless_uart()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {
        // 发送消息示例
        lcd_show_string(0, 0, "Press KEY_B to send");
        if (keymsg.key == KEY_B)
        {
            lcd_show_string(0, 1, "Sending data...");
            // 发送测试数据
            uint8 test_data[] = "TEST DATA\r\n";
            wireless_send_buffer(test_data, sizeof(test_data));
            lcd_show_string(0, 2, "Data sent!");
            system_delay_ms(500); // 等待发送完成
        }
        if (wireless_read_buffer(g_wireless_uart_buffer, WIRELESS_BUFFER_SIZE))
        {
            // 显示接收到的数据
            lcd_show_string(0, 3, "Received data:");
            lcd_show_string(0, 4, (const uint8 *)g_wireless_uart_buffer);
        }
        else
        {
            lcd_show_string(0, 3, "No data received");
        }
    }
    lcd_clear();
}

void test_key()
{
    lcd_clear();

    while (1)
    {
        lcd_show_string(0, 2, "Key pressed:");
        lcd_show_int(10, 2, keymsg.key, 2);
        lcd_show_string(0, 3, "Status:");
        lcd_show_int(10, 3, keymsg.status, 2);

        system_delay_ms(1); // 减少刷新频率
    }
    lcd_clear();
}

void test_sd_card()
{
    lcd_clear();
    lcd_show_string(0, 0, "SD Card Test");
    lcd_show_string(0, 1, "Initializing...");

    sd_result_t result = sd_init();
    if (result != SD_OK)
    {
        lcd_show_string(0, 2, "Init Failed!");
        lcd_show_string(0, 3, "Check connection");
        lcd_show_string(0, 7, "Press L to exit");
        while (keymsg.key != KEY_L)
        {
            system_delay_ms(100);
        }
        lcd_clear();
        return;
    }

    lcd_show_string(0, 2, "Init Success!");
    system_delay_ms(500);

    // Clear previous data
    sd_clear_data();

    lcd_clear();
    lcd_show_string(0, 0, "Writing data...");

    // Int
    uint32 int_value = 12345;
    result = sd_write_data((uint8 *)&int_value, sizeof(int_value), SD_WRITE_OVERRIDE);

    // String
    char str_value[] = "SD Card Test OK!";
    result = sd_write_data((uint8 *)str_value, strlen(str_value) + 1, SD_WRITE_APPEND);

    // Float
    float float_value = 3.14159;
    result = sd_write_data((uint8 *)&float_value, sizeof(float_value), SD_WRITE_APPEND);

    lcd_show_string(0, 1, "Write completed");
    system_delay_ms(500);

    lcd_clear();
    lcd_show_string(0, 0, "Reading data...");

    // Get total data size
    uint32 total_size = sd_get_data_size();
    lcd_show_string(0, 1, "Data size:");
    lcd_show_uint(10, 1, total_size, 5);

    // Allocate buffer for all data
    uint8 *buffer = (uint8 *)malloc(total_size);
    if (buffer == NULL)
    {
        lcd_show_string(0, 2, "Memory error!");
        system_delay_ms(1000);
        lcd_clear();
        return;
    }

    // Read
    uint32 read_size = 0;
    result = sd_read_data(buffer, total_size, &read_size);

    if (result != SD_OK)
    {
        lcd_show_string(0, 2, "Read failed!");
        free(buffer);
        system_delay_ms(1000);
        lcd_clear();
        return;
    }

    lcd_show_string(0, 2, "Read success!");

    // Display the data
    // Int
    uint32 *int_ptr = (uint32 *)buffer;
    lcd_show_string(0, 3, "Int:");
    lcd_show_uint(5, 3, *int_ptr, 5);

    // String
    char *str_ptr = (char *)(buffer + sizeof(uint32));
    lcd_show_string(0, 4, "Str:");
    lcd_show_string(5, 4, str_ptr);

    // Float
    float *float_ptr = (float *)(buffer + sizeof(uint32) + strlen(str_ptr) + 1);
    lcd_show_string(0, 5, "Float:");
    lcd_show_float(7, 5, *float_ptr, 3, 4);

    // Free memory
    free(buffer);

    // Wait for user to exit
    lcd_show_string(0, 7, "Press L to exit");
    while (keymsg.key != KEY_L)
    {
        system_delay_ms(100);
    }

    lcd_clear();
}

void test_ble6a20()
{
    lcd_clear();
    lcd_show_string(0, 0, "BLE6A20 Test");
    lcd_show_string(0, 1, "KEY_B: Toggle mode");
    lcd_show_string(0, 2, "KEY_L: Exit test");

    ble6a20_init(); // 初始化BLE6A20模块

    uint8 mode = 0;                    // 0: 接收模式, 1: 发送模式
    uint8 buffer[BLE6A20_BUFFER_SIZE]; // 接收缓冲区

    // 初始显示当前模式

    while (keymsg.key != KEY_L)
    {
        // 检测是否按下KEY_B切换模式
        if (keymsg.key == KEY_B)
        {
            mode = !mode; // 切换模式
            lcd_show_string(0, 3, "mode:");
            lcd_show_int(6, 3, mode, 3);
            system_delay_ms(200); // 防抖延时
        }

        // 根据当前模式执行相应操作
        if (mode == 0) // 发送模式
        {
            lcd_show_string(0, 4, "Sending data...");
            // 发送测试数据
            ble6a20_send_string("Hello BLE6A20!\r\n");
            lcd_show_string(0, 4, "Data sent!     ");
            system_delay_ms(1000); // 发送间隔
        }
        else // 接收模式
        {
            // 清空缓冲区
            memset(buffer, 0, BLE6A20_BUFFER_SIZE);

            // 接收数据
            uint32 len = ble6a20_read_buffer(buffer, BLE6A20_BUFFER_SIZE);
            if (len > 0)
            {
                lcd_show_string(0, 5, "Received:      ");
                printf("Received: %s", buffer); // 打印接收到的数据
            }
        }
    }
    lcd_clear();
}
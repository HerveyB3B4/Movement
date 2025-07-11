#include "test.h"
#include "attitude.h"
#include "guide.h"
#include "encoder.h"
#include "image.h"
#include "lcd.h"
#include "menu_input.h"
#include "motor.h"
#include "single_driver.h"
#include "velocity.h"
#include "zf_common_headfile.h"
#include "wireless.h"
#include "sd_card.h"
#include "receiver.h"
#include "detection.h"
#include "distance.h"
#include "diode.h"
#include "YawIntegral.h"
// #include "ins_core.h"
// #include "waypoint_manager.h"

// 定义静态变量，从栈移到数据段，避免栈溢出
static uint16_t s_edge_map[MT9V03X_W][MT9V03X_H];
#define WIRELESS_BUFFER_SIZE 256 // 定义无线UART缓冲区大小
static uint8 s_wireless_uart_buffer[WIRELESS_BUFFER_SIZE];

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
        lcd_show_float(0, 6, g_vel_motor.bottom_real, 5, 5);
        lcd_show_float(0, 7, g_vel_motor.bottom_filtered, 5, 5);
        pwm_set_duty(MOTOR_BOTTOM, 0);
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
        lcd_show_float(0, 1, PITCH, 3, 3);
        lcd_show_float(8, 1, PITCH - g_menu_manual_param.mechanicalPitchAngle * 0.1f, 3, 3);
        lcd_show_string(0, 2, "Row:");
        lcd_show_float(0, 3, ROLL, 3, 3);
        lcd_show_float(8, 3, ROLL - g_menu_manual_param.mechanicalRollAngle * 0.1f, 3, 3);
        lcd_show_string(0, 4, "Yaw:");
        lcd_show_float(0, 5, YAW, 3, 3);
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
        lcd_show_float(7, 7, g_vel_motor.bottom_real, 3, 2);

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
        lcd_show_float(7, 3, g_vel_motor.bottom_real, 3, 2);

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

void test_send_img()
{
    lcd_clear();
    wireless_assistant_init();
    while (keymsg.key != KEY_L)
    {
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            wireless_assistant_send_image(mt9v03x_image);
        }
    }
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

void test_receiver()
{
    lcd_clear();
    lcd_show_string(0, 0, "Receiver Test");

    while (keymsg.key != KEY_L)
    {
        lcd_show_int(0, 2, g_received_vel, 5);
    }

    lcd_clear();
}

void test_img_shoot()
{
    lcd_clear();

    lcd_show_string(0, 0, "Image Capture Test");
    system_delay_ms(1000);

    lcd_clear();

    sd_result_t sd_result = sd_init();
    if (sd_result != SD_OK)
    {
        lcd_show_string(0, 0, "SD Init Failed!");
        system_delay_ms(500);
    }
    else
    {
        lcd_show_string(0, 0, "SD Init Success!");
        sd_clear_data(); // 清除之前的数据
        system_delay_ms(500);
    }

    lcd_clear();

    static uint8 cnt = 0;
    while (keymsg.key != KEY_L)
    {
        if (mt9v03x_finish_flag != 0)
        {
            lcd_show_image(mt9v03x_image, MT9V03X_W, MT9V03X_H, 0);
            mt9v03x_finish_flag = 0;

            if (keymsg.key == KEY_B)
            {
                memcpy(s_edge_map, mt9v03x_image, MT9V03X_W * MT9V03X_H); // 暂且拿这个存
                sd_result = sd_write_data((uint8 *)s_edge_map, MT9V03X_IMAGE_SIZE, SD_WRITE_APPEND);
                if (sd_result == SD_OK)
                {
                    lcd_show_string(0, 7, "saved");
                    lcd_show_uint(8, 7, cnt++, 3);
                }
            }
        }
    }

    lcd_clear();
}

void test_line()
{
    lcd_clear();
    int pos = 0;
    uint8_t last_key = KEY_NONE; // 记录上一次按键，用于消抖
    while (keymsg.key != KEY_L)  // 主循环，L键退出
    {
        if ((keymsg.key == KEY_U || keymsg.key == KEY_D) && keymsg.key != last_key)
        {
            // 如果是新的按键，更新位置
            if (keymsg.key == KEY_D) // 向上移动线
            {
                pos += 1;
                if (pos >= MT9V03X_H)
                    pos = 0; // 限制在图像高度范围内
            }
            else if (keymsg.key == KEY_U) // 向下移动线
            {
                pos -= 1;
                if (pos < 0)
                    pos = MT9V03X_H - 1; // 限制在图像高度范围内
            }

            // 更新last_key，并进入等待按键释放状态
            last_key = keymsg.key;
        }
        else if (keymsg.key == KEY_NONE && last_key != KEY_NONE)
        {
            // 当按键释放时，重置last_key以允许下一次按键触发
            last_key = KEY_NONE;
        }

        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            draw_Hline(mt9v03x_image, pos, RGB565_WHITE); // 绘制水平线
            lcd_show_image(mt9v03x_image, MT9V03X_W, MT9V03X_H, 0);
            lcd_show_int(0, 7, pos, 3);
            lcd_show_float(5, 7, PITCH, 3, 3);
        }
    }
}

void test_yaw_integral()
{
    lcd_clear();
    integral_init(0.001f * PIT_CONTROL_T);
    while (keymsg.key != KEY_L)
    {
        imu_get_data(&g_imu_data);
        imu_remove_offset(&g_imu_data);
        integral_update(&g_imu_data.gyro.z);
        lcd_show_float(0, 0, g_imu_data.gyro.z, 3, 3);
        lcd_show_float(0, 1, integral_get_yaw(), 3, 3);
        system_delay_ms(1);
    }
    lcd_clear();
}

void test_encoder()
{
    lcd_clear();
    pit_disable(CCU60_CH0);
    encoder_clear_count(ENCODER_BOTTOM);
    while (keymsg.key != KEY_L)
    {
        int16 count = encoder_get_count(ENCODER_BOTTOM);
        lcd_show_int(0, 0, count, 5);
    }
    encoder_clear_count(ENCODER_BOTTOM);
    pit_enable(CCU60_CH0);
    lcd_clear();
}

void test_switch()
{
    lcd_clear();
    lcd_show_string(0, 0, "Switch Test");
    while (keymsg.key != KEY_L)
    {
        lcd_show_string(0, 1, "Switch 1:");
        lcd_show_int(9, 1, switch_get_state(SWITCH_1), 1);
        lcd_show_string(0, 2, "Switch 2:");
        lcd_show_int(9, 2, switch_get_state(SWITCH_2), 1);
        lcd_show_string(0, 3, "Switch 3:");
        lcd_show_int(9, 3, switch_get_state(SWITCH_3), 1);
        lcd_show_string(0, 4, "Switch 4:");
        lcd_show_int(9, 4, switch_get_state(SWITCH_4), 1);
    }
    lcd_clear();
}

void test_diode()
{
    lcd_clear();
    lcd_show_string(0, 0, "LED & Buzzer");
    lcd_show_string(0, 1, "KEY_U: val++");
    lcd_show_string(0, 2, "KEY_D: val--");
    lcd_show_string(0, 3, "KEY_R: switch");
    lcd_show_string(0, 4, "KEY_B: on");
    lcd_show_string(0, 5, "Press KEY_L to exit");
    uint8 cur = 0;
    uint8 val[3] = {0, 0, 0};
    while (keymsg.key != KEY_L)
    {
        if (keymsg.key == KEY_U)
        {
            val[cur]++;
        }
        else if (keymsg.key == KEY_D)
        {
            val[cur]--;
        }
        else if (keymsg.key == KEY_R)
        {
            cur = (cur + 1) % 3;
        }
        else if (keymsg.key == KEY_B)
        {
            uint8 i = DIODE_NUM;
            while (i--)
            {
                diode_set(i, val[0], val[1], val[2]);
                diode_on(i);
            }
        }
        lcd_show_uint(0, 6, val[0], 3);
        lcd_show_uint(5, 6, val[1], 3);
        lcd_show_uint(10, 6, val[2], 3);
        system_delay_ms(100);
    }
    uint8 i = DIODE_NUM;
    while (i--)
    {
        diode_off(i);
    }
    lcd_clear();
}

void test_dual_camera()
{
    lcd_clear();
    while (keymsg.key != KEY_L)
    {

        if (mt9v03x_finish_flag)
        {
            tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 0);
            mt9v03x_finish_flag = 0;
        }
        if (mt9v03x2_finish_flag)
        {
            tft180_show_gray_image(0, tft180_height_max - MT9V03X_H / 2, mt9v03x2_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 0);
            mt9v03x2_finish_flag = 0;
        }
    }
    lcd_clear();
}

void test_cpu_freq()
{
    lcd_clear();
    lcd_show_string(0, 0, "CPU Freq");
    while (keymsg.key != KEY_L)
    {
        lcd_show_string(0, 1, "CPU0: ");
        lcd_show_float(6, 1, IfxScuCcu_getCpuFrequency(IfxCpu_ResourceCpu_0) / 1e6, 5, 2);
        lcd_show_string(11, 1, " MHz");
        lcd_show_string(0, 2, "CPU1: ");
        lcd_show_float(6, 2, IfxScuCcu_getCpuFrequency(IfxCpu_ResourceCpu_1) / 1e6, 5, 2);
        lcd_show_string(11, 2, " MHz");
        lcd_show_string(0, 3, "CPU2: ");
        lcd_show_float(6, 3, IfxScuCcu_getCpuFrequency(IfxCpu_ResourceCpu_2) / 1e6, 5, 2);
        lcd_show_string(11, 3, " MHz");
    }
    lcd_clear();
}

// #define min(x, y) ((x) < (y) ? (x) : (y))

// uint8 txt[20];

// void test_navigation()
// {
//     ins_init(&g_ins_state);
//     waypoint_manager_init();

//     lcd_clear();

//     while (keymsg.key != KEY_L)
//     {
//         printf("%f | %f | %f | %f | %f | %f | %f | %f | %f\n", ROLL_ACC, PITCH_ACC, YAW_ACC, g_ins_state.pos_n, g_ins_state.pos_e, g_ins_state.pos_d, g_ins_state.vel_n, g_ins_state.vel_e, g_ins_state.vel_d);

//         if (keymsg.key == KEY_B)
//         {
//             waypoint_save_current_point(&g_ins_state);
//         }

//         lcd_show_string(0, 0, "pos_n");
//         lcd_show_float(8, 0, g_ins_state.pos_n, 5, 2);
//         lcd_show_string(0, 1, "pos_e");
//         lcd_show_float(8, 1, g_ins_state.pos_e, 5, 2);
//         lcd_show_string(0, 2, "pos_d");
//         lcd_show_float(8, 2, g_ins_state.pos_d, 5, 2);
//         lcd_show_string(0, 3, "vel_n");
//         lcd_show_float(8, 3, g_ins_state.vel_n, 5, 2);
//         lcd_show_string(0, 4, "vel_e");
//         lcd_show_float(8, 4, g_ins_state.vel_e, 5, 2);
//         lcd_show_string(0, 5, "vel_d");
//         lcd_show_float(8, 5, g_ins_state.vel_d, 5, 2);

//         system_delay_ms(10);
//     }

    lcd_clear();
}

void test_dual_camera_logic()
{
    lcd_clear();

    // 初始化状态机
    state_machine_context_t sm;
    state_machine_init(&sm);

    // 为前后摄像头分别创建二值化图像缓冲区
#pragma section all "cpu1_dsram"
    static uint8 binary_front[IMG_WIDTH * IMG_HEIGHT];
    static uint8 binary_rear[IMG_WIDTH * IMG_HEIGHT];
#pragma section all restore    

    while (keymsg.key != KEY_L)
    {
        // 等待两个摄像头的图像帧都准备好
        if (mt9v03x_finish_flag && mt9v03x2_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            mt9v03x2_finish_flag = 0;

            // 对两个摄像头的图像进行二值化
            binary_otsu(mt9v03x_image[0], binary_front);
            binary_otsu(mt9v03x2_image[0], binary_rear);

            // 运行状态机，传入两个图像
            state_machine_run(&sm, binary_front, binary_rear);

            // --- 可视化 ---
            connected_component_info active_target = sm.current_target;
            active_camera_t active_cam = sm.active_camera;

            // 在被选中的目标图像上绘制标定框
            if (active_target.bbox.area > 0)
            {
                uint8 *active_buffer = (active_cam == CAM_FRONT) ? binary_front : binary_rear;
                uint16 width = active_target.bbox.max_x - active_target.bbox.min_x;
                uint16 height = active_target.bbox.max_y - active_target.bbox.min_y;
                draw_rectangle(active_buffer, active_target.bbox.min_x, active_target.bbox.min_y, width, height, 128);
                draw_cross(active_buffer, active_target.center, 5, 200);
            }

            // 显示两个摄像头的图像 (缩小一半以并排显示)
            tft180_show_gray_image(0, 0, binary_front, MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 0);
            tft180_show_gray_image(0, tft180_height_max - MT9V03X_H / 2, binary_rear, MT9V03X_W, MT9V03X_H, MT9V03X_W / 2, MT9V03X_H / 2, 0);

            // 显示状态机信息
            char state_str[20];
            switch (sm.current_state)
            {
            case STATE_SEARCHING:
                strcpy(state_str, "SEARCH ");
                break;
            case STATE_APPROACHING_TARGET:
                strcpy(state_str, "APPROACH");
                break;
            case STATE_CIRCLING_TARGET:
                strcpy(state_str, " CIRCLE ");
                break;
            default:
                strcpy(state_str, " UNKNOWN");
                break;
            }
            lcd_show_string(12, 0, state_str);

            // 显示目标信息
            if (active_target.bbox.area > 0)
            {
                if (active_cam == CAM_FRONT)
                {
                    lcd_show_string(12, 1, "  FRONT ");
                }
                else
                {
                    lcd_show_string(12, 1, "  REAR  ");
                }
                lcd_show_string(12, 2, "X:      ");
                lcd_show_int(14, 2, active_target.center.x, 3);
                lcd_show_string(12, 3, "Y:      ");
                lcd_show_int(14, 3, active_target.center.y, 3);
            }
            else
            {
                lcd_show_string(12, 1, "No targe");
                lcd_show_string(12, 2, "t found ");
                lcd_show_string(12, 3, "in sight");
            }
        }
    }
    lcd_clear();
}

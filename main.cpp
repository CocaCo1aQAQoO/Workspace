#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "ms6dsv.h"
#include "kalman.h"
#include "ibus.h"
#include "led_driver.h"

const float LOOP_TIME_SEC = 0.01f;
const float RAD_TO_DEG = 57.29577951f;

int main() {
    std::cout << "=== 极简飞控启动 (MS6DSV + IBUS) ===" << std::endl;

    // 0. 初始化状态指示灯 (放最前面，让无人机一通电就亮起)
    StatusLED flightLed;
    if (!flightLed.init()) {
        std::cerr << "警告: LED 初始化失败，但不影响飞行，继续启动..." << std::endl;
    }
    flightLed.setYellow(); // 🟡 亮黄灯：代表系统正在进行自检

    // 1. 初始化姿态传感器
    MS6DSV imu("/dev/i2c-3", 0x6b);
    if (!imu.init()) {
        std::cerr << "错误: MS6DSV 初始化失败！" << std::endl;
        return -1;
    }
    KalmanFilter kalman_roll, kalman_pitch;

    // 2. 初始化接收机 (Milk-V Duo 的 GP3 引脚对应 UART1，即 ttyS1)
    IBus receiver;
    if (!receiver.init("/dev/ttyS1")) {
        std::cerr << "错误: 遥控器接收机串口初始化失败！" << std::endl;
        flightLed.setRed(); // 🔴 亮红灯：接收机丢失，禁止起飞
        return -1;
    }

    std::cout << ">>> 正在校准陀螺仪，请保持飞控绝对静止！(亮黄灯)" << std::endl;
    flightLed.setYellow(); // 校准期间亮黄灯警告不要碰

    float gyro_x_offset = 0.0f, gyro_y_offset = 0.0f, gyro_z_offset = 0.0f;
    float accel_roll_offset = 0.0f, accel_pitch_offset = 0.0f;
    const int CALIBRATION_SAMPLES = 200;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        imu.read_raw_accel(ax, ay, az);
        imu.read_raw_gyro(gx, gy, gz);

        gyro_x_offset += static_cast<float>(gx) / 16.4f;
        gyro_y_offset += static_cast<float>(gy) / 16.4f;
        gyro_z_offset += static_cast<float>(gz) / 16.4f;

        float ax_f = static_cast<float>(ax) / 2048.0f;
        float ay_f = static_cast<float>(ay) / 2048.0f;
        float az_f = static_cast<float>(az) / 2048.0f;
        accel_roll_offset += atan2(-ax_f, az_f) * RAD_TO_DEG;
        accel_pitch_offset += atan2(ay_f, sqrt(ax_f * ax_f + az_f * az_f)) * RAD_TO_DEG;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gyro_x_offset /= CALIBRATION_SAMPLES;
    gyro_y_offset /= CALIBRATION_SAMPLES;
    gyro_z_offset /= CALIBRATION_SAMPLES;
    accel_roll_offset /= CALIBRATION_SAMPLES;
    accel_pitch_offset /= CALIBRATION_SAMPLES;

    std::cout << ">>> 校准完成！X零偏: " << gyro_x_offset 
              << ", Y零偏: " << gyro_y_offset << std::endl;

    std::cout << "初始化完成，进入 100Hz 主循环..." << std::endl;
    flightLed.setBlue(); // 校准完变蓝灯，等待起飞

    auto next_loop_time = std::chrono::steady_clock::now();
    bool is_crashed = false;

    while (true) {
        next_loop_time += std::chrono::milliseconds(10);

        // --- A. 读取遥控器输入 ---
        receiver.update(); // 瞬间读取串口缓冲区的最新数据
        
        // 富斯 i6 遥控器通道映射 (美国手 Mode 2):
        // CH1: Roll (副翼，右手横向)
        // CH2: Pitch (升降，右手纵向)
        // CH3: Throttle (油门，左手纵向)
        // CH4: Yaw (方向，左手横向)
        uint16_t rx_roll     = receiver.get_channel(1);
        uint16_t rx_pitch    = receiver.get_channel(2);
        uint16_t rx_throttle = receiver.get_channel(3);
        uint16_t rx_yaw      = receiver.get_channel(4);

        // 将 1000~2000 的遥控器原始数据，归一化为 -1.0 ~ 1.0 的百分比
        float stick_roll  = (rx_roll - 1500) / 500.0f;
        float stick_pitch = (rx_pitch - 1500) / 500.0f;
        float stick_yaw   = (rx_yaw - 1500) / 500.0f;
        
        // 油门比较特殊，直接取 0 ~ 1000 的物理混控基准值
        float base_throttle = rx_throttle > 1000 ? (rx_throttle - 1000) : 0.0f; 

        // --- B. 读取姿态传感器 ---
        int16_t ax, ay, az, gx, gy, gz;
        imu.read_raw_accel(ax, ay, az);
        imu.read_raw_gyro(gx, gy, gz);

        float accel_x = static_cast<float>(ax) / 2048.0f;
        float accel_y = static_cast<float>(ay) / 2048.0f;
        float accel_z = static_cast<float>(az) / 2048.0f;

        float gyro_x = (static_cast<float>(gx) / 16.4f) - gyro_x_offset;
        float gyro_y = (static_cast<float>(gy) / 16.4f) - gyro_y_offset;

        // 先计算原始角度，再减去开机测量的绝对物理倾角偏置
        float raw_accel_pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * RAD_TO_DEG;
        float raw_accel_roll  = atan2(-accel_x, accel_z) * RAD_TO_DEG;

        float accel_pitch = raw_accel_pitch - accel_pitch_offset;
        float accel_roll  = raw_accel_roll - accel_roll_offset;

        // 滤波器现在的输入起点为 0 度
        float estimated_roll  = kalman_roll.get_angle(accel_roll, gyro_x, LOOP_TIME_SEC);
        float estimated_pitch = kalman_pitch.get_angle(accel_pitch, gyro_y, LOOP_TIME_SEC);

        // --- C. 🌟 LED 智能状态机逻辑 ---
        // 逻辑 1：坠机检测保护
        if (std::abs(estimated_roll) > 75.0f || std::abs(estimated_pitch) > 75.0f) {
            is_crashed = true;
            flightLed.setRed();     // 🔴 姿态倾角过大，判定为翻车，亮红灯报警！
            base_throttle = 0.0f;   // (预留) 强行切断电机动力
        } 
        // 逻辑 2：正常飞行状态指示
        else if (!is_crashed) {
            // 油门在最低点附近 (未起飞)
            if (rx_throttle < 1010) {
                flightLed.setBlue();  // 🔵 待机安全状态
            } 
            // 只要一推油门，立刻进入战斗状态
            else {
                flightLed.setGreen(); // 🟢 战斗状态，正在飞行！
            }
        }

        // --- D. 桌面调试输出 (每隔 0.1 秒打印一次) ---
        static int count = 0;
        if (count++ % 10 == 0) {
            std::cout << "动力: " << base_throttle 
                      << " | 摇杆(Roll/Pitch/Yaw): " << stick_roll << ", " << stick_pitch << ", " << stick_yaw
                      << " | 姿态(R/P): " << estimated_roll << "°, " << estimated_pitch << "°" << std::endl;
        }

        std::this_thread::sleep_until(next_loop_time);
    }

    flightLed.setOff();
    return 0;
}
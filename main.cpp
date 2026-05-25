#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "ms6dsv.h"
#include "kalman.h"
#include "ibus.h"  // 👇 新增遥控器驱动

const float LOOP_TIME_SEC = 0.01f;
const float RAD_TO_DEG = 57.29577951f;

int main() {
    std::cout << "=== 极简飞控启动 (MS6DSV + IBUS) ===" << std::endl;

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
    }

    std::cout << "初始化完成，进入 100Hz 主循环..." << std::endl;
    auto next_loop_time = std::chrono::steady_clock::now();

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
        float gyro_x = static_cast<float>(gx) / 16.4f;
        float gyro_y = static_cast<float>(gy) / 16.4f;

        float accel_pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * RAD_TO_DEG;
        float accel_roll  = atan2(-accel_x, accel_z) * RAD_TO_DEG;

        float estimated_roll  = kalman_roll.get_angle(accel_roll, gyro_x, LOOP_TIME_SEC);
        float estimated_pitch = kalman_pitch.get_angle(accel_pitch, gyro_y, LOOP_TIME_SEC);

        // --- C. 桌面调试输出 (每隔 0.1 秒打印一次) ---
        static int count = 0;
        if (count++ % 10 == 0) {
            std::cout << "油门: " << rx_throttle 
                      << " | Roll杆: " << stick_roll 
                      << " | 当前倾角: " << estimated_roll << "°" << std::endl;
        }

        std::this_thread::sleep_until(next_loop_time);
    }
    return 0;
}
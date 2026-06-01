#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "ms6dsv.h"
#include "kalman.h"
#include "ibus.h"
#include "led_driver.h"
#include "pid.h"
#include "pwm_esc.h" 

const float LOOP_TIME_SEC = 0.01f;
const float RAD_TO_DEG = 57.29577951f;

int main() {
    std::cout << "=== 极简飞控启动 (完整闭环: IMU + IBUS + PID + PWM) ===" << std::endl;

    // 0. 初始化状态指示灯
    StatusLED flightLed;
    flightLed.init();
    flightLed.setYellow();

    // 1. 初始化姿态传感器
    MS6DSV imu("/dev/i2c-3", 0x6a);
    if (!imu.init()) {
        std::cerr << "错误: MS6DSV 初始化失败！" << std::endl;
        return -1;
    }
    KalmanFilter kalman_roll, kalman_pitch;

    // 2. 初始化接收机
    IBus receiver;
    if (!receiver.init("/dev/ttyS1")) {
        std::cerr << "错误: 遥控器接收机串口初始化失败！" << std::endl;
        flightLed.setRed(); 
        return -1;   
    }

    std::cout << ">>> 正在初始化无刷电调..." << std::endl;
    PWMEsc esc1(8, 2); // M1: 前左电机
    PWMEsc esc2(8, 3); // M2: 前右电机
    PWMEsc esc3(4, 0); // M3: 后左电机
    PWMEsc esc4(4, 1); // M4: 后右电机

    if (!esc1.init() || !esc2.init() || !esc3.init() || !esc4.init()) {
        std::cerr << "致命错误：电调 PWM 接口初始化失败！" << std::endl;
        flightLed.setRed();
        return -1;
    }
    esc1.setThrottle(1000); esc2.setThrottle(1000);
    esc3.setThrottle(1000); esc4.setThrottle(1000);

    PIDController pid_roll(1.5f, 0.0f, 0.5f, 50.0f);
    PIDController pid_pitch(1.5f, 0.0f, 0.5f, 50.0f);
    PIDController pid_yaw(2.0f, 0.0f, 0.0f, 50.0f);

    std::cout << ">>> 正在校准陀螺仪，请保持绝对静止！" << std::endl;
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

    std::cout << "初始化完成，进入 100Hz 闭环控制循环..." << std::endl;
    flightLed.setBlue(); 

    auto next_loop_time = std::chrono::steady_clock::now();
    bool is_crashed = false;
    
    float estimated_yaw = 0.0f; 
    float target_yaw = 0.0f; 

    while (true) {
        next_loop_time += std::chrono::milliseconds(10);

        // --- A. 读取遥控器输入 (加入死区过滤 & 排空堆积) ---
        // 🌟 核心修复 1：连读 3 次，彻底排空 Linux 串口缓冲区的积压历史数据，消除延迟！
        for (int i = 0; i < 3; i++) {
            receiver.update(); 
        }
        
        uint16_t rx_roll     = receiver.get_channel(1); 
        uint16_t rx_pitch    = receiver.get_channel(2); 
        uint16_t rx_throttle = receiver.get_channel(3); 
        uint16_t rx_yaw      = receiver.get_channel(4); 
        uint16_t rx_arm_switch = receiver.get_channel(5);

        bool is_armed = (rx_arm_switch > 1500); 

        const uint16_t DEADBAND = 15;
        if (rx_roll > 1500 - DEADBAND && rx_roll < 1500 + DEADBAND) rx_roll = 1500;
        if (rx_pitch > 1500 - DEADBAND && rx_pitch < 1500 + DEADBAND) rx_pitch = 1500;
        if (rx_yaw > 1500 - DEADBAND && rx_yaw < 1500 + DEADBAND) rx_yaw = 1500;

        float stick_roll  = (rx_roll - 1500) / 500.0f;
        float stick_pitch = (rx_pitch - 1500) / 500.0f;
        float stick_yaw   = (rx_yaw - 1500) / 500.0f;
        
        // 🌟 核心修复 2：把底部死区抬高到 1030，确保推到底时电机能干净利落地停下
        float base_throttle = 0.0f;
        if (rx_throttle > 1030) {
            base_throttle = rx_throttle - 1000.0f;
        }

        // --- B. 读取当前真实姿态 (Current) ---
        int16_t ax, ay, az, gx, gy, gz;
        imu.read_raw_accel(ax, ay, az);
        imu.read_raw_gyro(gx, gy, gz);

        float accel_x = static_cast<float>(ax) / 2048.0f;
        float accel_y = static_cast<float>(ay) / 2048.0f;
        float accel_z = static_cast<float>(az) / 2048.0f;

        float gyro_x = (static_cast<float>(gx) / 16.4f) - gyro_x_offset;
        float gyro_y = (static_cast<float>(gy) / 16.4f) - gyro_y_offset;
        float gyro_z = (static_cast<float>(gz) / 16.4f) - gyro_z_offset; 

        float accel_pitch = (atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * RAD_TO_DEG) - accel_pitch_offset;
        float accel_roll  = (atan2(-accel_x, accel_z) * RAD_TO_DEG) - accel_roll_offset;

        float estimated_roll  = kalman_roll.get_angle(accel_roll, gyro_x, LOOP_TIME_SEC);
        float estimated_pitch = kalman_pitch.get_angle(accel_pitch, gyro_y, LOOP_TIME_SEC);
        estimated_yaw += gyro_z * LOOP_TIME_SEC; 

        // --- C. 将遥控器信号转换为目标姿态 (Target) ---
        float target_roll = stick_roll * 30.0f;
        float target_pitch = stick_pitch * 30.0f;
        
        if (is_armed && base_throttle > 10.0f) {
            target_yaw += stick_yaw * 1.5f; 
        } else {
            target_yaw = estimated_yaw;
            pid_roll.reset();
            pid_pitch.reset();
            pid_yaw.reset();
        }

        // --- D. PID 核心运算 ---
        float pid_out_roll  = pid_roll.update(target_roll, estimated_roll, LOOP_TIME_SEC);
        float pid_out_pitch = pid_pitch.update(target_pitch, estimated_pitch, LOOP_TIME_SEC);
        float pid_out_yaw   = pid_yaw.update(target_yaw, estimated_yaw, LOOP_TIME_SEC);

        // --- E. 🌟重构：绝对优先级状态机与动力输出 ---
        if (std::abs(estimated_roll) > 75.0f || std::abs(estimated_pitch) > 75.0f) {
            is_crashed = true;
        } 
        
        // 优先级 1：坠机（绝对锁死，亮红灯）
        if (is_crashed) {
            flightLed.setRed();      
            esc1.setThrottle(1000);
            esc2.setThrottle(1000);
            esc3.setThrottle(1000);
            esc4.setThrottle(1000);
        } 
        // 优先级 2：未解锁或零油门待机（安全切断，亮蓝灯）
        else if (!is_armed || base_throttle < 10.0f) {
            flightLed.setBlue();     
            esc1.setThrottle(1000);
            esc2.setThrottle(1000);
            esc3.setThrottle(1000);
            esc4.setThrottle(1000);
        } 
        // 优先级 3：正常战斗飞行（激活混控，亮绿灯）
        else {
            flightLed.setGreen(); 
            
            // 外旋 (Props Out) 专属 X型四轴混控矩阵
            float out1 = 1000.0f + base_throttle - pid_out_pitch + pid_out_roll + pid_out_yaw;
            float out2 = 1000.0f + base_throttle - pid_out_pitch - pid_out_roll - pid_out_yaw;
            float out3 = 1000.0f + base_throttle + pid_out_pitch + pid_out_roll - pid_out_yaw;
            float out4 = 1000.0f + base_throttle + pid_out_pitch - pid_out_roll + pid_out_yaw;

            // 🌟 核心修复 3：去掉 1050 的强制怠速，恢复底线为 1000（允许停转）。
            // 彻底杜绝 C++ 负数转换导致的 65535 满转抽风！
            auto clamp_pwm = [](float out) -> uint16_t {
                if (out < 1000.0f) return 1000;
                if (out > 2000.0f) return 2000;
                return static_cast<uint16_t>(out);
            };

            esc1.setThrottle(clamp_pwm(out1));
            esc2.setThrottle(clamp_pwm(out2));
            esc3.setThrottle(clamp_pwm(out3));
            esc4.setThrottle(clamp_pwm(out4));
        }

        // --- F. 降频打印 ---
        static int count = 0;
        if (count++ % 10 == 0) {
            std::cout << (is_crashed ? "[🔴坠机锁死] " : (is_armed ? "[🟢已解锁] " : "[🔵已上锁] "))
                      << "油门: " << base_throttle 
                      << " | 目标角(R/P): " << target_roll << "°, " << target_pitch << "°"
                      << " | 实际角(R/P): " << estimated_roll << "°, " << estimated_pitch << "°" << std::endl;
        }

        std::this_thread::sleep_until(next_loop_time);
    }

    flightLed.setOff();
    return 0;
}
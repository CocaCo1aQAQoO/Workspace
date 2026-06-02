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
#include "bmp280.h"

const float LOOP_TIME_SEC = 0.01f;
const float RAD_TO_DEG = 57.29577951f;

int main() {
    std::cout << "=== 🚀 极简飞控 V1.0 (实战飞行版: 外旋 + 死区 + Airmode) ===" << std::endl;

    StatusLED flightLed;
    flightLed.init();
    flightLed.setYellow();

    MS6DSV imu("/dev/i2c-3", 0x6a);
    if (!imu.init()) {
        std::cerr << "错误: MS6DSV 初始化失败！" << std::endl;
        return -1;
    }
    KalmanFilter kalman_roll, kalman_pitch;

    BMP280 barometer("/dev/i2c-3", 0x76);
    if (!barometer.init()) {
        std::cerr << "警告: 气压计初始化失败！" << std::endl;
    } else {
        std::cout << ">>> BMP 气压计初始化成功！" << std::endl;
    }

    IBus receiver;
    if (!receiver.init("/dev/ttyS1")) {
        std::cerr << "错误: 接收机丢失！" << std::endl;
        flightLed.setRed(); 
        return -1;   
    }

    PWMEsc esc1(8, 2); // M1: 前左 (外旋: CCW)
    PWMEsc esc2(8, 3); // M2: 前右 (外旋: CW)
    PWMEsc esc3(4, 0); // M3: 后左 (外旋: CW)
    PWMEsc esc4(4, 1); // M4: 后右 (外旋: CCW)

    if (!esc1.init() || !esc2.init() || !esc3.init() || !esc4.init()) {
        flightLed.setRed();
        return -1;
    }
    esc1.setThrottle(1000); esc2.setThrottle(1000);
    esc3.setThrottle(1000); esc4.setThrottle(1000);

    // 初始试飞 PID 建议参数 (稳妥起见，降低了 P，增加了 D)
    PIDController pid_roll(1.2f, 0.0f, 0.6f, 50.0f);
    PIDController pid_pitch(1.2f, 0.0f, 0.6f, 50.0f);
    PIDController pid_yaw(2.0f, 0.0f, 0.0f, 50.0f);

    std::cout << ">>> 正在校准，请保持飞机绝对水平静止..." << std::endl;
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

    flightLed.setBlue(); 
    auto next_loop_time = std::chrono::steady_clock::now();
    bool is_crashed = false;
    float estimated_yaw = 0.0f, target_yaw = 0.0f; 

    while (true) {
        next_loop_time += std::chrono::milliseconds(10);

        // 连读排空缓冲区，消除延迟
        for (int i = 0; i < 3; i++) receiver.update(); 
        
        // 【通道映射】严格按照日本手 Mode 1 设定
        uint16_t rx_roll     = receiver.get_channel(1); // CH1: 右手左右
        uint16_t rx_pitch    = receiver.get_channel(2); // CH2: 左手上下
        uint16_t rx_throttle = receiver.get_channel(3); // CH3: 右手上下
        uint16_t rx_yaw      = receiver.get_channel(4); // CH4: 左手左右
        uint16_t rx_arm_switch = receiver.get_channel(5);

        bool is_armed = (rx_arm_switch > 1500); 

        // 摇杆死区过滤
        const uint16_t DEADBAND = 15;
        if (rx_roll > 1500 - DEADBAND && rx_roll < 1500 + DEADBAND) rx_roll = 1500;
        if (rx_pitch > 1500 - DEADBAND && rx_pitch < 1500 + DEADBAND) rx_pitch = 1500;
        if (rx_yaw > 1500 - DEADBAND && rx_yaw < 1500 + DEADBAND) rx_yaw = 1500;

        float stick_roll  = (rx_roll - 1500) / 500.0f;
        float stick_pitch = (rx_pitch - 1500) / 500.0f;
        float stick_yaw   = (rx_yaw - 1500) / 500.0f;
        float base_throttle = (rx_throttle > 1030) ? (rx_throttle - 1000.0f) : 0.0f;

        // 传感器读取与姿态解算
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

        float target_roll = stick_roll * 30.0f;
        float target_pitch = stick_pitch * 30.0f;
        
        if (is_armed && base_throttle > 10.0f) {
            target_yaw += stick_yaw * 1.5f; 
        } else {
            target_yaw = estimated_yaw;
            pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset();
        }

        float pid_out_roll  = pid_roll.update(target_roll, estimated_roll, LOOP_TIME_SEC);
        float pid_out_pitch = pid_pitch.update(target_pitch, estimated_pitch, LOOP_TIME_SEC);
        float pid_out_yaw   = pid_yaw.update(target_yaw, estimated_yaw, LOOP_TIME_SEC);

        // 坠机保护阈值：真实飞行设定为 75 度物理极限
        if (std::abs(estimated_roll) > 75.0f || std::abs(estimated_pitch) > 75.0f) {
            is_crashed = true;
        } 
        
        if (is_crashed) {
            flightLed.setRed();      
            esc1.setThrottle(1000); esc2.setThrottle(1000);
            esc3.setThrottle(1000); esc4.setThrottle(1000);
        } 
        else if (!is_armed || base_throttle < 10.0f) {
            flightLed.setBlue();     
            esc1.setThrottle(1000); esc2.setThrottle(1000);
            esc3.setThrottle(1000); esc4.setThrottle(1000);
        } 
        else {
            flightLed.setGreen(); 
            // 🌟 外旋 (Props Out) 混控矩阵
            float out1 = 1000.0f + base_throttle - pid_out_pitch + pid_out_roll + pid_out_yaw;
            float out2 = 1000.0f + base_throttle - pid_out_pitch - pid_out_roll - pid_out_yaw;
            float out3 = 1000.0f + base_throttle + pid_out_pitch + pid_out_roll - pid_out_yaw;
            float out4 = 1000.0f + base_throttle + pid_out_pitch - pid_out_roll + pid_out_yaw;

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

        // 🌟 新增：读取气压计并降频打印测试
        float temp = 0.0f, press = 0.0f, alt = 0.0f;
        barometer.read_sensor(temp, press, alt);

        static int count = 0;
        if (count++ % 10 == 0) { // 每 10 个循环（即每 0.1 秒）打印一次，防止刷屏卡顿
            std::cout << "高度: " << alt << " 米 | 气压: " << press << " Pa | 温度: " << temp << " °C" << std::endl;
        }
        // ==========================================
        
        std::this_thread::sleep_until(next_loop_time);
    }
    return 0;
}
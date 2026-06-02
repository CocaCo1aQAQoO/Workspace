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
    
    // 🌟 新增：Z轴高度 PID 控制器 (用于定高)
    PIDController pid_alt(20.0f, 0.0f, 15.0f, 100.0f); 

    // 🌟 新增：定高与失控保护的状态变量
    bool is_alt_hold = false;
    float hover_throttle = 0.0f;
    float target_alt = 0.0f;

    // ==========================================
    // 🌟 脱机飞行核心：开机待机监听锁
    std::cout << ">>> 硬件底层就绪！等待遥控器 SWA (CH5) 拨下以唤醒飞控..." << std::endl;
    flightLed.setYellow(); // 黄灯常亮代表“通电待机中”

    while (true) {
        // 清空串口积压并读取最新通道数据
        for (int i = 0; i < 3; i++) receiver.update(); 
        uint16_t rx_arm_switch = receiver.get_channel(5);

        // 如果侦测到 SWA 拨杆被拨下 (富斯遥控器拨下通常输出 2000)
        if (rx_arm_switch > 1500) {
            break; // 💥 冲破待机锁，进入下方真实的校准和飞行控制流！
        }

        // 待机期间，持续喂给电调 1000us 的最低信号，防止电调超时哔哔报警
        esc1.setThrottle(1000); esc2.setThrottle(1000);
        esc3.setThrottle(1000); esc4.setThrottle(1000);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    // ==========================================
    
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

        
        // ==========================================
        // 读取气压计数据 (需放在前面以供 PID 使用)
        float temp = 0.0f, press = 0.0f, alt = 0.0f;
        barometer.read_sensor(temp, press, alt);
        // ==========================================

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

        // 🌟 核心修正：真正的失控保护 (Failsafe)
        // 正常最低油门是 1000。只有遥控器关机、信号丢失或接收机断线时，值才会跌至 950 以下。
        bool is_failsafe = (rx_throttle < 950);

        // 🌟 核心 2：定高悬停 (Altitude Hold) 逻辑
        float pid_out_alt = 0.0f;
        // 设定油门中位死区 (1450~1550)。如果你把右手摇杆放在中间，飞机自动接管高度！
        if (rx_throttle > 1450 && rx_throttle < 1550 && is_armed && base_throttle > 100.0f) {
            if (!is_alt_hold) {
                target_alt = alt; // 瞬间锁定当前气压高度
                hover_throttle = base_throttle; // 锁定你当前的物理推力
                is_alt_hold = true;
            }
            pid_out_alt = pid_alt.update(target_alt, alt, LOOP_TIME_SEC);
            base_throttle = hover_throttle + pid_out_alt; // PID 接管油门
        } else {
            is_alt_hold = false;
            pid_alt.reset(); // 退出定高时，清空历史误差积分
        }

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

        if (std::abs(estimated_roll) > 75.0f || std::abs(estimated_pitch) > 75.0f) {
            is_crashed = true;
        } 
        
        // 🚨 优先级 0：失控保护 (最高绝对指令)
        if (is_failsafe) {
            flightLed.setRed(); // 红灯长亮代表失控
            // TODO: 未来可在此处实现 "缓慢迫降" 逻辑。当前为了安全，直接停转。
            esc1.setThrottle(1000); esc2.setThrottle(1000);
            esc3.setThrottle(1000); esc4.setThrottle(1000);
        }
        // 🚨 优先级 1：坠机锁死
        else if (is_crashed) {
            flightLed.setRed();      
            esc1.setThrottle(1000); esc2.setThrottle(1000);
            esc3.setThrottle(1000); esc4.setThrottle(1000);
        } 
        // 🔵 优先级 2：安全上锁 / 怠速防误触
        else if (!is_armed || base_throttle < 10.0f) {
            flightLed.setBlue();     
            esc1.setThrottle(1000); esc2.setThrottle(1000);
            esc3.setThrottle(1000); esc4.setThrottle(1000);
        } 
        // 🟢 优先级 3：正常战斗飞行 (激活外旋混控)
        else {
            flightLed.setGreen(); 
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

        // 📝 默认状态输出 (每 0.1 秒刷新一次，干净、专业)
        static int count = 0;
        if (count++ % 10 == 0) { 
            std::cout << (is_failsafe ? "[⚠️ 失控]" : (is_crashed ? "[❌ 坠机]" : (is_armed ? "[🚀 战斗]" : "[🔒 安全]")))
                      << (is_alt_hold ? " [定高 ON] " : " [定高 OFF]")
                      << " 油门: " << (int)base_throttle 
                      << " | 高度: " << alt << "m"
                      << " | 倾角(R/P): " << (int)estimated_roll << "°," << (int)estimated_pitch << "°" 
                      << std::endl;
        }

        std::this_thread::sleep_until(next_loop_time);
    }
    return 0;
}
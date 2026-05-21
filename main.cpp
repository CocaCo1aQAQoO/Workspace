/**
 * 基于 Milk-V Duo 的 RISC-V 飞控系统主程序
 * 硬件平台: Milk-V Duo 256 (主控)
 * 核心功能: 姿态解算、闭环 PID 控制、UDP 遥测数据回传
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <algorithm>

// 引入我们之前写好的各个模块头文件
#include "pid.h"
#include "imu.h"
#include "rc_math.h"
#include "led_driver.h"
#include "ultrasonic.h"
#include "bmp280.h"
#include "dht11.h"
#include "logger.h"
#include "buzzer.h"
#include "adc_battery.h"

// --- 全局配置参数 ---
const float LOOP_TIME_SEC = 0.01f;      // 目标循环时间 10ms (100Hz) 
const float LOW_BATT_THRESHOLD = 3.2f;  // 低电压报警阈值 3.2V

// --- 地站 UDP 配置 ---
const char* GS_IP = "192.168.1.100";    // 未来替换为你的电脑 IP
const int GS_PORT = 8888;               // FastAPI 监听的端口

// 简单的 UDP 发送工具函数
void send_telemetry(int sock, struct sockaddr_in& addr, float pitch, float roll, float yaw, float alt, float temp, float humi) {
    std::string msg = "{\"pitch\":" + std::to_string(pitch) + 
                      ",\"roll\":" + std::to_string(roll) + 
                      ",\"yaw\":" + std::to_string(yaw) + 
                      ",\"alt\":" + std::to_string(alt) + 
                      ",\"temp\":" + std::to_string(temp) + 
                      ",\"humi\":" + std::to_string(humi) + "}";
    sendto(sock, msg.c_str(), msg.length(), 0, (struct sockaddr*)&addr, sizeof(addr));
}

int main() {
    std::cout << "========== Milk-V Duo 飞控系统启动 ==========" << std::endl;

    // 1. 初始化外设驱动
    RGBLed status_led(14, 15, 16);
    status_led.init();
    status_led.set_state(DroneState::BOOTING);

    IMUSensor imu;
    // 假设硬件连接在 I2C 1 号总线
    if (!imu.init("/dev/i2c-1")) {
        std::cerr << "警告: IMU 初始化失败，请检查接线！" << std::endl;
        status_led.set_state(DroneState::ERROR);
    }

    Ultrasonic front_sonar(17, 18);
    if (!front_sonar.init()) {
        std::cerr << "警告: 超声波模块初始化失败！" << std::endl;
    }

    BMP280 barometer;
    if (!barometer.init("/dev/i2c-1")) {
        std::cerr << "警告: 气压计初始化失败！" << std::endl;
    }

    DHT11 dht_sensor(19);
    if (!dht_sensor.init()) {
        std::cerr << "警告: DHT11 初始化失败！" << std::endl;
    }

    // 👇 新增：初始化蜂鸣器和电压检测模块
    Buzzer drone_buzzer(20);
    if (!drone_buzzer.init()) {
        std::cerr << "警告: 蜂鸣器驱动初始化失败！" << std::endl;
    }

    AdcBattery battery_sensor(1, 3.3f, 5.0f); // 1号ADC，基准3.3V，分压比5.0
    if (!battery_sensor.init()) {
        std::cerr << "警告: ADC电压检测节点初始化失败！" << std::endl;
    }

    // 用于限制 DHT11 读取频率的定时器和全局数据
    auto last_dht_time = std::chrono::steady_clock::now();
    float current_humidity = 0.0f; 

    // 2. 初始化 PID 控制器 (参数需实地试飞调参)
    PIDController roll_pid(1.2f, 0.05f, 0.3f, 50.0f);
    PIDController pitch_pid(1.2f, 0.05f, 0.3f, 50.0f);
    PIDController yaw_pid(2.5f, 0.0f, 0.0f, 50.0f);
    PIDController alt_pid(15.0f, 0.1f, 5.0f, 200.0f); 
    float target_altitude = 1.0f;

    // 3. 初始化 UDP 网络 Socket
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in gs_addr;
    gs_addr.sin_family = AF_INET;
    gs_addr.sin_port = htons(GS_PORT);
    inet_pton(AF_INET, GS_IP, &gs_addr.sin_addr);

    // 初始化黑匣子 (文件将保存在 Milk-V Duo 的当前运行目录下)
    DataLogger blackbox;
    if (!blackbox.init("flight_log.csv")) {
        std::cerr << "警告: 黑匣子 SD 卡文件创建失败！" << std::endl;
    }
    // 记录系统启动的零点时间，用于生成时间戳
    auto system_start_time = std::chrono::steady_clock::now();

    std::cout << "系统初始化完成，进入 100Hz 控制主循环..." << std::endl;
    status_led.set_state(DroneState::STANDBY);

    // 记录循环开始时间
    auto next_loop_time = std::chrono::steady_clock::now();

    // ================= 飞控核心主循环 =================
    while (true) {
        // 更新下一次期望的循环时间点 (+10ms)
        next_loop_time += std::chrono::milliseconds(10);

        // --- A. 传感器读取与姿态解算 ---
        imu.update(LOOP_TIME_SEC); // 内部执行读取和互补滤波 

        // --- B. 读取遥控器输入 (硬件未到前先用模拟值) ---
        // 假设通过串口或独立引脚读取到了原始值，归一化到 -1.0 ~ 1.0
        float raw_roll_stick  = 0.0f; 
        float raw_pitch_stick = 0.0f;
        float raw_yaw_stick   = 0.0f;
        float base_throttle   = 400.0f; // 基础油门

        // 进行死区和指数曲线(Expo)处理
        float target_roll  = RCMath::process_channel(raw_roll_stick, 0.05f, 0.6f) * 30.0f; // 最大倾角30度
        float target_pitch = RCMath::process_channel(raw_pitch_stick, 0.05f, 0.6f) * 30.0f;
        float target_yaw   = RCMath::process_channel(raw_yaw_stick, 0.05f, 0.6f) * 45.0f;

        // 超声波避障逻辑拦截
        float distance = front_sonar.get_distance_m();
        if (distance > 0.0f && distance < 1.0f) { 
            // 发现前方 1 米内有障碍物，无视遥控器推杆，强行给一个向后的目标角度进行刹车
            target_pitch = -15.0f; 
        }
        
        // 👇 修改：C. 系统状态机与安全逻辑 (接入真实电池读数与蜂鸣器)
        float battery_voltage = battery_sensor.read_voltage(); 
        
        // 防御策略：电压 <= 2.0V 认为是 USB 供电调试，不报警，状态为 STANDBY
        if (battery_voltage <= 2.0f) {
            status_led.set_state(DroneState::STANDBY);
            drone_buzzer.set(false);
        } 
        // 低压保护触发
        else if (battery_voltage < LOW_BATT_THRESHOLD) {
            status_led.set_state(DroneState::LOW_BATTERY);
            drone_buzzer.set(true); // 📢 触发物理蜂鸣器尖叫！
            
            // 触发低压保护：主动缓慢降低基础油门
            base_throttle *= 0.8f; 
        } 
        // 正常带电状态
        else {
            status_led.set_state(DroneState::ARMED_FLYING);
            drone_buzzer.set(false); // 闭嘴
        }

        // 修复后的气压计定高逻辑
        float current_alt = barometer.get_relative_altitude();
        float current_temp = 0.0f;
        float current_press = 0.0f; 
        barometer.read_sensor_data(current_temp, current_press); 
        
        // 只有当遥控器油门推到中间悬停区时，才介入自动定高
        if (base_throttle > 300.0f && base_throttle < 600.0f) {
            float throttle_adjust = alt_pid.update(target_altitude, current_alt, LOOP_TIME_SEC);
            base_throttle += throttle_adjust; // 自动推拉油门
        }

        // --- D. PID 闭环控制计算 ---
        float roll_out  = roll_pid.update(target_roll, imu.roll, LOOP_TIME_SEC);
        float pitch_out = pitch_pid.update(target_pitch, imu.pitch, LOOP_TIME_SEC);
        float yaw_out   = yaw_pid.update(target_yaw, imu.yaw, LOOP_TIME_SEC);

        // --- E. 电机混控 (X型四轴) ---
        float motor1 = base_throttle - roll_out + pitch_out - yaw_out;
        float motor2 = base_throttle + roll_out + pitch_out + yaw_out;
        float motor3 = base_throttle + roll_out - pitch_out - yaw_out;
        float motor4 = base_throttle - roll_out - pitch_out + yaw_out;

        // 限幅保护，防止 PWM 越界
        motor1 = std::clamp(motor1, 0.0f, 1000.0f);
        motor2 = std::clamp(motor2, 0.0f, 1000.0f);
        motor3 = std::clamp(motor3, 0.0f, 1000.0f);
        motor4 = std::clamp(motor4, 0.0f, 1000.0f);

        // TODO: 调用底层 PWM 驱动输出给电调 (硬件连线后编写)
        // set_motor_pwm(motor1, motor2, motor3, motor4);

        // 每 2 秒读取一次温湿度，绝不阻塞主控制环！
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_dht_time).count() >= 2) {
            float temp_h, temp_t;
            if (dht_sensor.read_data(temp_h, temp_t)) {
                current_humidity = temp_h;
            }
            last_dht_time = now;
        }

        // --- F. 地面站遥测数据发送 ---
        send_telemetry(udp_sock, gs_addr, imu.pitch, imu.roll, imu.yaw, current_alt, current_temp, current_humidity);

        // 计算当前飞行时间，并写入黑匣子
        float flight_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - system_start_time).count() / 1000.0f;
        blackbox.log_frame(flight_time, imu.pitch, imu.roll, imu.yaw, current_alt, motor1, motor2, motor3, motor4);

        // --- G. 严格时钟同步 ---
        // 如果当前时间早于期望时间，就休眠剩下的时间；如果超时则直接进入下一轮
        std::this_thread::sleep_until(next_loop_time);
    }

    close(udp_sock);
    return 0;
}
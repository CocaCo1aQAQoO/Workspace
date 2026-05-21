#include "ultrasonic.h"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

Ultrasonic::Ultrasonic(int trig, int echo) : trig_pin(trig), echo_pin(echo) {}

bool Ultrasonic::export_gpio(int pin) {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) return false;
    export_file << pin;
    export_file.close();
    return true;
}

bool Ultrasonic::set_direction(int pin, const std::string& dir) {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream dir_file(path);
    if (!dir_file.is_open()) return false;
    dir_file << dir;
    dir_file.close();
    return true;
}

bool Ultrasonic::set_value(int pin, int value) {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream val_file(path);
    if (!val_file.is_open()) return false;
    val_file << value;
    val_file.close();
    return true;
}

// 读取 GPIO 当前电平 (0 或 1)
int Ultrasonic::get_value(int pin) {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ifstream val_file(path);
    if (!val_file.is_open()) return -1;
    int value;
    val_file >> value;
    val_file.close();
    return value;
}

bool Ultrasonic::init() {
    export_gpio(trig_pin);
    export_gpio(echo_pin);
    usleep(50000); // 等待系统导出完成

    if (!set_direction(trig_pin, "out") || !set_direction(echo_pin, "in")) {
        std::cerr << "超声波初始化失败: 无法配置 GPIO 模式" << std::endl;
        return false;
    }
    set_value(trig_pin, 0); // 确保触发脚初始为低电平
    return true;
}

float Ultrasonic::get_distance_m() {
    // 1. 发送 10 微秒的高电平触发信号
    set_value(trig_pin, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    set_value(trig_pin, 0);

    auto start_time = std::chrono::high_resolution_clock::now();
    auto timeout_start = start_time;

    // 2. 死循环等待 Echo 引脚变高 (超声波发射)
    while (get_value(echo_pin) == 0) {
        start_time = std::chrono::high_resolution_clock::now();
        // 设置一个超时保护，防止死锁 (例如传感器没接好)
        if (std::chrono::duration_cast<std::chrono::milliseconds>(start_time - timeout_start).count() > 50) {
            return -1.0f; // 50ms 还没回音，说明前方无障碍或传感器断线
        }
    }

    // 3. 死循环等待 Echo 引脚变低 (收到回波)
    auto end_time = start_time;
    while (get_value(echo_pin) == 1) {
        end_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() > 50) {
            return -1.0f; // 测距超出最大范围 (超过 8 米)
        }
    }

    // 4. 计算持续时间 (微秒)
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    // 5. 将时间换算为距离：声速约为 340m/s，也就是 0.00034m/us
    // 距离 = (时间 * 0.00034) / 2
    float distance_meters = (duration * 0.00034f) / 2.0f;
    return distance_meters;
}
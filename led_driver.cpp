
#include "led_driver.h"
#include <fstream>
#include <iostream>
#include <unistd.h> // 提供 usleep 延时函数

RGBLed::RGBLed(int r, int g, int b) : pin_r(r), pin_g(g), pin_b(b) {}

// 将 GPIO 编号导出到用户空间
bool RGBLed::export_gpio(int pin) {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) return false;
    export_file << pin;
    export_file.close();
    return true;
}

// 设置 GPIO 为输出模式 (out)
bool RGBLed::set_direction(int pin, const std::string& dir) {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream dir_file(path);
    if (!dir_file.is_open()) return false;
    dir_file << dir;
    dir_file.close();
    return true;
}

// 设置 GPIO 高低电平 (1 或 0)
bool RGBLed::set_value(int pin, int value) {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream val_file(path);
    if (!val_file.is_open()) return false;
    val_file << value;
    val_file.close();
    return true;
}

bool RGBLed::init() {
    int pins[] = {pin_r, pin_g, pin_b};
    for (int pin : pins) {
        export_gpio(pin);
        // 系统导出 GPIO 需要极短的时间，稍微延时一下防止找不到文件
        usleep(50000); 
        if (!set_direction(pin, "out")) {
            std::cerr << "LED 初始化失败: 无法设置引脚 " << pin << " 为输出模式" << std::endl;
            return false;
        }
        set_value(pin, 0); // 初始状态全灭
    }
    return true;
}

RGBLed::~RGBLed() {
    // 程序退出时熄灭所有灯
    set_color(0, 0, 0);
    // 严谨的做法是写入 /sys/class/gpio/unexport 释放引脚，这里为了代码简洁略去
}

void RGBLed::set_color(int r_val, int g_val, int b_val) {
    set_value(pin_r, r_val);
    set_value(pin_g, g_val);
    set_value(pin_b, b_val);
}

void RGBLed::set_state(DroneState state) {
    switch (state) {
        case DroneState::BOOTING:
            set_color(1, 1, 0); // 红+绿=黄
            break;
        case DroneState::STANDBY:
            set_color(0, 1, 0); // 绿灯
            break;
        case DroneState::ARMED_FLYING:
            set_color(0, 0, 1); // 蓝灯
            break;
        case DroneState::LOW_BATTERY:
            set_color(1, 0, 0); // 红灯
            break;
        case DroneState::ERROR:
            set_color(1, 0, 0); // 红灯
            break;
    }
}
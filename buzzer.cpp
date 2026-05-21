#include "buzzer.h"
#include <fstream>
#include <iostream>
#include <unistd.h>

Buzzer::Buzzer(int pin) : pin_(pin), is_initialized_(false) {
    gpio_path_ = "/sys/class/gpio/gpio" + std::to_string(pin_) + "/";
}

Buzzer::~Buzzer() {
    set(false); // 析构时确保蜂鸣器闭嘴
}

bool Buzzer::export_gpio() {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) return false;
    export_file << pin_;
    return true;
}

bool Buzzer::set_direction_out() {
    std::ofstream dir_file(gpio_path_ + "direction");
    if (!dir_file.is_open()) return false;
    dir_file << "out";
    return true;
}

bool Buzzer::init() {
    // 1. 尝试导出 GPIO（如果已经导出过，可能会失败，所以先检查目录是否存在）
    if (access(gpio_path_.c_str(), F_OK) == -1) {
        if (!export_gpio()) {
            std::cerr << "错误: 无法导出 GPIO " << pin_ << std::endl;
            return false;
        }
        // 给系统一点时间来创建虚拟文件节点
        usleep(100000); 
    }

    // 2. 设置为输出模式
    if (!set_direction_out()) {
        std::cerr << "错误: 无法设置 GPIO " << pin_ << " 为输出方向" << std::endl;
        return false;
    }

    is_initialized_ = true;
    return true;
}

void Buzzer::set(bool on) {
    if (!is_initialized_) return;
    std::ofstream value_file(gpio_path_ + "value");
    if (value_file.is_open()) {
        value_file << (on ? "1" : "0");
    }
}
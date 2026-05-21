
#include "dht11.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

DHT11::DHT11(int pin) : data_pin(pin) {}

bool DHT11::export_gpio() {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) return false;
    export_file << data_pin;
    export_file.close();
    return true;
}

bool DHT11::set_direction(const std::string& dir) {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(data_pin) + "/direction";
    std::ofstream dir_file(path);
    if (!dir_file.is_open()) return false;
    dir_file << dir;
    dir_file.close();
    return true;
}

bool DHT11::set_value(int value) {
    std::string path = "/sys/class/gpio/gpio" + std::to_string(data_pin) + "/value";
    std::ofstream val_file(path);
    if (!val_file.is_open()) return false;
    val_file << value;
    val_file.close();
    return true;
}

bool DHT11::init() {
    export_gpio();
    usleep(50000);
    // 初始状态设为输出高电平，让总线空闲
    if (!set_direction("out")) return false;
    set_value(1);
    return true;
}

bool DHT11::read_data(float& humidity, float& temperature) {
    int data[5] = {0, 0, 0, 0, 0};

    // 1. 发送起始信号：主机拉低至少 18ms，然后拉高
    set_direction("out");
    set_value(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    set_value(1);
    
    // 立即切换为输入模式，准备接收传感器响应
    set_direction("in");

    // 为了实现极速读取，使用底层 open 和 read
    std::string path = "/sys/class/gpio/gpio" + std::to_string(data_pin) + "/value";
    int fd = open(path.c_str(), O_RDONLY);
    if (fd < 0) return false;

    // 极速读取 GPIO 电平的 Lambda 函数
    auto get_val = [&]() -> int {
        char c;
        lseek(fd, 0, SEEK_SET);
        if (read(fd, &c, 1) != 1) return -1;
        return c - '0';
    };

    // 2. 等待 DHT11 响应 (拉低 80us，拉高 80us)
    auto wait_for_state = [&](int state, int timeout_us) -> bool {
        auto start = std::chrono::high_resolution_clock::now();
        while (get_val() != state) {
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::microseconds>(now - start).count() > timeout_us) {
                return false; // 超时退出
            }
        }
        return true;
    };

    // 等待传感器响应低电平，然后等待它变回高电平，再等待数据开始的低电平
    if (!wait_for_state(0, 40)) { close(fd); return false; }
    if (!wait_for_state(1, 100)) { close(fd); return false; }
    if (!wait_for_state(0, 100)) { close(fd); return false; }

    // 3. 读取 40 bit 数据
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            // 每 bit 都是以 50us 低电平开始
            if (!wait_for_state(1, 100)) { close(fd); return false; }
            
            // 记录高电平持续时间
            auto bit_start = std::chrono::high_resolution_clock::now();
            if (!wait_for_state(0, 100)) { close(fd); return false; }
            auto bit_end = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(bit_end - bit_start).count();
            
            // 如果高电平时间 > 40us，说明这一位是 '1' (一般是 70us)；否则是 '0' (一般 26-28us)
            data[i] <<= 1;
            if (duration > 40) {
                data[i] |= 1;
            }
        }
    }
    
    close(fd);

    // 4. 数据校验: 前四个字节相加的末 8 位应该等于第五个校验字节
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        humidity = data[0] + data[1] / 10.0f;
        temperature = data[2] + data[3] / 10.0f;
        return true;
    }
    
    return false;
}
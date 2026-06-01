#include "pwm_esc.h"
#include <fstream>
#include <iostream>
#include <unistd.h>

PWMEsc::PWMEsc(int chip, int channel) : chipNum(chip), channelNum(channel), isInitialized(false) {
    pwmPath = "/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/pwm" + std::to_string(channelNum);
}

PWMEsc::~PWMEsc() {
    if (isInitialized) {
        setThrottle(1000); // 退出时强行拉低到安全油门
    }
}

void PWMEsc::writeToFile(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (file.is_open()) {
        file << value << std::endl;
        file.close();
    }
}

bool PWMEsc::init() {
    // 1. 导出 PWM 通道
    writeToFile("/sys/class/pwm/pwmchip" + std::to_string(chipNum) + "/export", std::to_string(channelNum));
    usleep(100000); // 等待内核创建节点

    // 2. 设置周期 (50Hz = 20,000,000 纳秒)
    writeToFile(pwmPath + "/period", "20000000");

    // 3. 设置初始占空比 (1000us = 1,000,000 纳秒，代表最低油门/停转)
    writeToFile(pwmPath + "/duty_cycle", "1000000");

    // 4. 开启 PWM 输出
    writeToFile(pwmPath + "/enable", "1");
    
    isInitialized = true;
    return true;
}

void PWMEsc::setThrottle(uint16_t throttle_us) {
    if (!isInitialized) return;
    // 限制范围在 1000 到 2000 微秒之间，防止烧毁
    if (throttle_us < 1000) throttle_us = 1000;
    if (throttle_us > 2000) throttle_us = 2000;
    
    // 转换为纳秒
    std::string duty_ns = std::to_string(throttle_us * 1000);
    writeToFile(pwmPath + "/duty_cycle", duty_ns);
}
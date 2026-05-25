#include "adc_battery.h"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>

AdcBattery::AdcBattery(int channel, float vref, float division_ratio) 
    : channel_(channel), vref_(vref), division_ratio_(division_ratio) {
    // 💡 替换为 Milk-V 专有的驱动节点路径
    adc_path_ = "/sys/class/cvi-saradc/cvi-saradc0/device/cv_saradc";
}

bool AdcBattery::init() {
    // 检查专有文件是否存在，确保 insmod 已经执行
    if (access(adc_path_.c_str(), F_OK) == -1) {
        std::cerr << "错误: 找不到专有 ADC 节点，请确保执行了 insmod cv181x_saradc.ko" << std::endl;
        return false;
    }
    return true;
}

float AdcBattery::read_voltage() {
    // 必须用底层 POSIX O_RDWR 方式打开，C++ fstream 会被这里的奇葩逻辑搞晕
    int fd = open(adc_path_.c_str(), O_RDWR);
    if (fd < 0) return 0.0f;

    // 1. 原厂奇葩逻辑：先向文件写入你要测的通道号 (例如 "1")
    std::string chan_str = std::to_string(channel_);
    if (write(fd, chan_str.c_str(), chan_str.length()) < 0) {
        close(fd);
        return 0.0f;
    }

    // 2. 然后从同一个文件里读取返回的字符串
    char buffer[32];
    memset(buffer, 0, sizeof(buffer));
    int len = read(fd, buffer, sizeof(buffer) - 1);
    close(fd);

    if (len > 0) {
        // 将读取到的字符串转换为整型原始值 (0 ~ 4095)
        int raw_adc = std::stoi(buffer);
        
        // 3. 物理电压换算
        float pin_voltage = (static_cast<float>(raw_adc) / 4095.0f) * vref_;
        float battery_voltage = pin_voltage * division_ratio_;
        return battery_voltage;
    }
    
    return 0.0f; 
}
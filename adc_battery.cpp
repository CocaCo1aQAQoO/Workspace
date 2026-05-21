#include "adc_battery.h"
#include <fstream>
#include <iostream>
#include <unistd.h>

AdcBattery::AdcBattery(int channel, float vref, float division_ratio) 
    : channel_(channel), vref_(vref), division_ratio_(division_ratio) {
    // 拼接 Linux IIO 系统的原生 ADC 文件路径
    adc_path_ = "/sys/bus/iio/devices/iio:device0/in_voltage" + std::to_string(channel_) + "_raw";
}

bool AdcBattery::init() {
    // 检查 Linux ADC 驱动节点是否存在
    if (access(adc_path_.c_str(), F_OK) == -1) {
        std::cerr << "错误: 找不到 Linux ADC 节点: " << adc_path_ << "，请检查内核驱动是否加载" << std::endl;
        return false;
    }
    return true;
}

float AdcBattery::read_voltage() {
    std::ifstream adc_file(adc_path_);
    int raw_adc = 0;
    
    if (adc_file >> raw_adc) {
        // 1. 将 0~4095 的原始整数转换为引脚上的实际物理电压 (0~3.3V)
        float pin_voltage = (static_cast<float>(raw_adc) / 4095.0f) * vref_;
        
        // 2. 通过分压比还原为真实的电池电压
        float battery_voltage = pin_voltage * division_ratio_;
        return battery_voltage;
    }
    
    // 如果读取失败，返回 0.0f 防止误判成满电
    return 0.0f; 
}
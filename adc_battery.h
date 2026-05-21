#ifndef ADC_BATTERY_H
#define ADC_BATTERY_H

#include <string>

class AdcBattery {
private:
    int channel_;
    std::string adc_path_;
    float vref_;           // 基准电压，Milk-V Duo 通常是 3.3V
    float division_ratio_; // 分压板比例，Keyes 模块为 5.0

public:
    // channel 传入 1 代表 ADC1
    AdcBattery(int channel, float vref = 3.3f, float division_ratio = 5.0f);
    
    bool init();
    
    // 读取并返回转换后的真实电池电压值 (V)
    float read_voltage();
};

#endif // ADC_BATTERY_H
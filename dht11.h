
#ifndef DHT11_H
#define DHT11_H

#include <string>

class DHT11 {
private:
    int data_pin;

    // 基础 GPIO 操作
    bool export_gpio();
    bool set_direction(const std::string& dir);
    bool set_value(int value);

public:
    // 构造函数，传入数据引脚
    DHT11(int pin);
    
    // 初始化引脚
    bool init();

    // 读取温湿度 (返回 true 表示校验成功，数据有效)
    bool read_data(float& humidity, float& temperature);
};

#endif
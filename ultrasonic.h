#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <string>

class Ultrasonic {
private:
    int trig_pin;
    int echo_pin;

    // 底层 sysfs 文件操作
    bool export_gpio(int pin);
    bool set_direction(int pin, const std::string& dir);
    bool set_value(int pin, int value);
    int get_value(int pin);

public:
    // 构造函数，传入 Trig 和 Echo 对应的 GPIO 编号
    Ultrasonic(int trig, int echo);
    
    // 初始化引脚
    bool init();

    // 触发并获取前方距离，返回值单位：米 (m)
    // 如果超时或读取失败，返回 -1.0f
    float get_distance_m();
};

#endif
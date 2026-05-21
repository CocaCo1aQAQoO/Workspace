#ifndef BUZZER_H
#define BUZZER_H

#include <string>

class Buzzer {
private:
    int pin_;
    std::string gpio_path_;
    bool is_initialized_;

    bool export_gpio();
    bool set_direction_out();

public:
    // 传入物理 GPIO 编号，比如 20
    Buzzer(int pin);
    ~Buzzer();

    // 初始化：导出引脚并设置为输出模式
    bool init();
    
    // 控制开关：true 鸣叫，false 闭嘴
    void set(bool on);
};

#endif // BUZZER_H
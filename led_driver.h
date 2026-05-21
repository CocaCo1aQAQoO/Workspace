#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include <string>

// 定义无人机的状态枚举
enum class DroneState {
    BOOTING,        // 启动中 (黄灯常亮)
    STANDBY,        // 待机未解锁 (绿灯慢闪/常亮)
    ARMED_FLYING,   // 解锁飞行中 (蓝灯常亮)
    LOW_BATTERY,    // 低电压报警 (红灯快闪/常亮)
    ERROR           // 系统故障 (红灯常亮)
};

class RGBLed {
private:
    int pin_r;
    int pin_g;
    int pin_b;

    // 底层 Linux sysfs 操作辅助函数
    bool export_gpio(int pin);
    bool set_direction(int pin, const std::string& dir);
    bool set_value(int pin, int value);

public:
    // 构造函数，传入 R, G, B 对应的 GPIO 编号
    RGBLed(int r, int g, int b);
    
    // 初始化引脚
    bool init();

    // 释放引脚控制权
    ~RGBLed();

    // 核心接口：根据无人机状态自动切换颜色
    void set_state(DroneState state);

    // 基础接口：直接控制三色亮灭 (0为灭，1为亮)
    // 注意：如果是共阳极 LED，可能需要反转逻辑 (0为亮，1为灭)
    void set_color(int r_val, int g_val, int b_val);
};

#endif
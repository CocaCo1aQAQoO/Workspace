#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include <string>

class StatusLED {
private:
    int pinR;
    int pinG;
    int pinB;
    bool isInitialized;

    // 内部私有方法：底层文件操作与内核寻址
    void writeToFile(const std::string& path, const std::string& value);
    void exportPin(int pin);
    int findGpioABase(); // 黑客级自动寻址函数

public:
    StatusLED();  // 构造函数
    ~StatusLED(); // 析构函数

    // 初始化硬件
    bool init();

    // 核心变色函数
    void setColor(int r, int g, int b);

    // 语义化快捷指令 (供主程序飞控逻辑调用)
    void setRed();    // 故障/未解锁
    void setGreen();  // 准备起飞
    void setBlue();   // 等待遥控器
    void setYellow(); // 警告
    void setPurple(); // 自定义
    void setCyan();   // GPS定位
    void setWhite();  // 探照灯
    void setOff();    // 熄灭
};

#endif // LED_DRIVER_H
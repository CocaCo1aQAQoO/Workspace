#include "led_driver.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstdlib>
#include <dirent.h> // 🌟 引入 C++ 原生目录扫描库

StatusLED::StatusLED() : pinR(-1), pinG(-1), pinB(-1), isInitialized(false) {}

StatusLED::~StatusLED() {
    if (isInitialized) {
        setOff(); 
    }
}

// 🌟 核心升级 1：放弃脆弱的 Shell 命令，使用 C++ 原生扫描内核目录寻找基址
int StatusLED::findGpioABase() {
    // 强制复用引脚打开大门
    system("duo-pinmux -w GP16/GP16 >/dev/null 2>&1");
    system("duo-pinmux -w GP17/GP17 >/dev/null 2>&1");
    system("duo-pinmux -w GP18/GP18 >/dev/null 2>&1");

    DIR *dir;
    struct dirent *ent;
    // 扫描 /sys/class/gpio/ 目录
    if ((dir = opendir("/sys/class/gpio")) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string dirName = ent->d_name;
            // 找到包含 "gpiochip" 的文件夹
            if (dirName.find("gpiochip") != std::string::npos) {
                std::string labelPath = "/sys/class/gpio/" + dirName + "/label";
                std::ifstream labelFile(labelPath);
                if (labelFile.is_open()) {
                    std::string label;
                    labelFile >> label;
                    // 如果文件内容包含 3020000 (GPIOA 的硬件地址)
                    if (label.find("3020000") != std::string::npos) {
                        std::string numStr = dirName.substr(8); // 提取 "gpiochip" 后面的数字
                        closedir(dir);
                        return std::stoi(numStr);
                    }
                }
            }
        }
        closedir(dir);
    }
    return -1;
}

// 🌟 核心升级 2：修复换行符，确保 Sysfs 写入被内核确认
void StatusLED::writeToFile(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (file.is_open()) {
        file << value << std::endl; // std::endl 包含换行并强制刷新缓冲区！
        file.close();
    }
}

void StatusLED::exportPin(int pin) {
    // 🌟 核心升级 3：听你的！先清除缓存（强制注销）
    writeToFile("/sys/class/gpio/unexport", std::to_string(pin));
    usleep(20000); // 给内核 20 毫秒的时间释放之前的僵尸进程

    // 重新干净地初始化
    writeToFile("/sys/class/gpio/export", std::to_string(pin));
    usleep(50000); // 必须延时，等待内核创建好全新的 value 和 direction 节点

    // 重新强行设为输出模式
    writeToFile("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction", "out");
}

bool StatusLED::init() {
    int baseAddr = findGpioABase();
    if (baseAddr == -1) {
        std::cerr << "[LED 驱动] ❌ 严重错误：无法找到 GPIOA 基址！" << std::endl;
        return false;
    }

    pinR = baseAddr + 23;
    pinG = baseAddr + 24;
    pinB = baseAddr + 22;

    std::cout << "[LED 驱动] ✅ 缓存已清除！成功接管引脚: R=" << pinR << ", G=" << pinG << ", B=" << pinB << std::endl;

    exportPin(pinR);
    exportPin(pinG);
    exportPin(pinB);
    
    isInitialized = true;
    setOff();
    return true;
}

void StatusLED::setColor(int r, int g, int b) {
    if (!isInitialized) return;
    writeToFile("/sys/class/gpio/gpio" + std::to_string(pinR) + "/value", std::to_string(r));
    writeToFile("/sys/class/gpio/gpio" + std::to_string(pinG) + "/value", std::to_string(g));
    writeToFile("/sys/class/gpio/gpio" + std::to_string(pinB) + "/value", std::to_string(b));
}

void StatusLED::setRed()    { setColor(1, 0, 0); }
void StatusLED::setGreen()  { setColor(0, 1, 0); }
void StatusLED::setBlue()   { setColor(0, 0, 1); }
void StatusLED::setYellow() { setColor(1, 1, 0); }
void StatusLED::setPurple() { setColor(1, 0, 1); }
void StatusLED::setCyan()   { setColor(0, 1, 1); }
void StatusLED::setWhite()  { setColor(1, 1, 1); }
void StatusLED::setOff()    { setColor(0, 0, 0); }
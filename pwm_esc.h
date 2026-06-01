#ifndef PWM_ESC_H
#define PWM_ESC_H

#include <string>
#include <cstdint>

class PWMEsc {
public:
    // 传入 chip 编号和 channel 编号
    PWMEsc(int chip, int channel);
    ~PWMEsc();

    bool init();
    // 传入 1000 ~ 2000 的微秒值 (1000 停转，2000 满油)
    void setThrottle(uint16_t throttle_us);

private:
    int chipNum;
    int channelNum;
    bool isInitialized;
    std::string pwmPath;
    
    void writeToFile(const std::string& path, const std::string& value);
};

#endif
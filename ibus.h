#ifndef IBUS_H
#define IBUS_H

#include <cstdint>

class IBus {
private:
    int fd;
    uint16_t channels[10];  // 存放解析后的通道数据 (通常范围 1000 ~ 2000)
    uint8_t packet_buffer[32];
    int buffer_index;

public:
    IBus();
    ~IBus();
    
    // 初始化串口，传入 "/dev/ttyS1"
    bool init(const char* port);
    
    // 在主循环中高频调用，非阻塞抓取并解析数据
    void update();
    
    // 获取通道数值 (ch: 1~10)
    uint16_t get_channel(int ch) const;
};

#endif
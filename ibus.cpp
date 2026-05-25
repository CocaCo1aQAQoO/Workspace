#include "ibus.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

IBus::IBus() : fd(-1), buffer_index(0) {
    // 默认给个安全值，特别是油门(通道3)给最低 1000
    for(int i = 0; i < 10; i++) channels[i] = 1500;
    channels[2] = 1000; 
}

IBus::~IBus() {
    if (fd >= 0) close(fd);
}

bool IBus::init(const char* port) {
    // 以读写、非阻塞模式打开串口
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "错误: 无法打开遥控器串口 " << port << std::endl;
        return false;
    }

    struct termios tty;
    tcgetattr(fd, &tty);

    // IBUS 协议要求 115200 波特率
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 8位数据位，无校验，1位停止位 (8N1)
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    // 关闭各种自动回显和特殊字符处理，纯净透传二进制
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &tty);
    return true;
}

void IBus::update() {
    if (fd < 0) return;

    uint8_t buf[64];
    // 非阻塞读取：能读多少读多少，读不到立马返回
    int n = read(fd, buf, sizeof(buf));
    
    if (n > 0) {
        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];
            
            // 状态机：寻找 IBUS 数据包的同步头 0x20 和 0x40
            if (buffer_index == 0 && b != 0x20) continue;
            if (buffer_index == 1 && b != 0x40) { buffer_index = 0; continue; }

            packet_buffer[buffer_index++] = b;

            // 凑齐 32 个字节，开始校验
            if (buffer_index == 32) {
                uint16_t checksum = 0xFFFF;
                for (int j = 0; j < 30; j++) checksum -= packet_buffer[j];
                
                uint16_t rx_checksum = (packet_buffer[31] << 8) | packet_buffer[30];

                // 校验通过，提取通道数据！(低字节在前，高字节在后)
                if (checksum == rx_checksum) {
                    for (int ch = 0; ch < 10; ch++) {
                        channels[ch] = (packet_buffer[3 + ch * 2] << 8) | packet_buffer[2 + ch * 2];
                    }
                }
                // 重置状态机，准备迎接下一个包
                buffer_index = 0; 
            }
        }
    }
}

uint16_t IBus::get_channel(int ch) const {
    if (ch >= 1 && ch <= 10) return channels[ch - 1];
    return 1500;
}
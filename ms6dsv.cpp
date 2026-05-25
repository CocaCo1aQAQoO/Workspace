#include "ms6dsv.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

MS6DSV::MS6DSV(const std::string& bus, uint8_t address) 
    : i2c_bus(bus), dev_addr(address), file_fd(-1) {}

MS6DSV::~MS6DSV() {
    if (file_fd >= 0) {
        close(file_fd);
    }
}

bool MS6DSV::init() {
    // 1. 打开 I2C 总线文件
    file_fd = open(i2c_bus.c_str(), O_RDWR);
    if (file_fd < 0) {
        std::cerr << "Failed to open the i2c bus: " << i2c_bus << std::endl;
        return false;
    }

    // 2. 绑定设备地址 (0x6b)
    if (ioctl(file_fd, I2C_SLAVE, dev_addr) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave." << std::endl;
        return false;
    }

    // 3. 验证设备身份 (读取 WHO_AM_I)
    uint8_t who_am_i = 0;
    if (read_registers(MS6DSV_WHO_AM_I, &who_am_i, 1)) {
        std::cout << "MS6DSV WHO_AM_I: 0x" << std::hex << (int)who_am_i << std::dec << std::endl;
    } else {
        std::cerr << "Failed to read WHO_AM_I." << std::endl;
        return false;
    }

    // 4. 唤醒传感器并配置工作模式
    // 开启加速度计: 104Hz, 16g 量程 (具体配置需查阅数据手册)
    write_register(MS6DSV_CTRL1_XL, 0x44); 
    // 开启陀螺仪: 104Hz, 2000 dps 量程
    write_register(MS6DSV_CTRL2_G, 0x4C);  

    std::cout << "MS6DSV Initialized Successfully!" << std::endl;
    return true;
}

bool MS6DSV::write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(file_fd, buffer, 2) != 2) {
        return false;
    }
    return true;
}

bool MS6DSV::read_registers(uint8_t reg, uint8_t* buffer, int length) {
    // I2C 读操作通常需要先写要读取的寄存器地址，然后再读数据
    if (write(file_fd, &reg, 1) != 1) {
        return false;
    }
    if (read(file_fd, buffer, length) != length) {
        return false;
    }
    return true;
}

void MS6DSV::read_raw_accel(int16_t& ax, int16_t& ay, int16_t& az) {
    uint8_t buffer[6];
    if (read_registers(MS6DSV_OUTX_L_A, buffer, 6)) {
        // 将高八位和低八位拼接成 16 位有符号整数
        ax = (int16_t)((buffer[1] << 8) | buffer[0]);
        ay = (int16_t)((buffer[3] << 8) | buffer[2]);
        az = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
}

void MS6DSV::read_raw_gyro(int16_t& gx, int16_t& gy, int16_t& gz) {
    uint8_t buffer[6];
    if (read_registers(MS6DSV_OUTX_L_G, buffer, 6)) {
        gx = (int16_t)((buffer[1] << 8) | buffer[0]);
        gy = (int16_t)((buffer[3] << 8) | buffer[2]);
        gz = (int16_t)((buffer[5] << 8) | buffer[4]);
    }
}
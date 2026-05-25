#ifndef MS6DSV_H
#define MS6DSV_H

#include <stdint.h>
#include <string>

// MS6DSV 关键寄存器地址 (请根据实际数据手册微调，此处为ST系列通用标准)
#define MS6DSV_WHO_AM_I      0x0F
#define MS6DSV_CTRL1_XL      0x10  // 加速度计控制寄存器
#define MS6DSV_CTRL2_G       0x11  // 陀螺仪控制寄存器
#define MS6DSV_OUTX_L_G      0x22  // 陀螺仪 X 轴低 8 位 (起始地址)
#define MS6DSV_OUTX_L_A      0x28  // 加速度计 X 轴低 8 位 (起始地址)

class MS6DSV {
public:
    MS6DSV(const std::string& bus, uint8_t address);
    ~MS6DSV();

    bool init();
    void read_raw_accel(int16_t& ax, int16_t& ay, int16_t& az);
    void read_raw_gyro(int16_t& gx, int16_t& gy, int16_t& gz);

private:
    std::string i2c_bus;
    uint8_t dev_addr;
    int file_fd;

    bool write_register(uint8_t reg, uint8_t value);
    bool read_registers(uint8_t reg, uint8_t* buffer, int length);
};

#endif
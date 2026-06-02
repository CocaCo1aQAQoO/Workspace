#include "bmp280.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

BMP280::BMP280(const std::string& bus, int address) 
    : fd(-1), bus_name(bus), device_address(address), reference_pressure(101325.0f) {}

BMP280::~BMP280() {
    if (fd >= 0) close(fd);
}

bool BMP280::write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(fd, buf, 2) == 2;
}

bool BMP280::read_registers(uint8_t reg, uint8_t* data, int length) {
    if (write(fd, &reg, 1) != 1) return false;
    return read(fd, data, length) == length;
}

void BMP280::read_calibration_data() {
    uint8_t calib[24];
    read_registers(0x88, calib, 24);
    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];
}

bool BMP280::init() {
    fd = open(bus_name.c_str(), O_RDWR);
    if (fd < 0) return false;
    if (ioctl(fd, I2C_SLAVE, device_address) < 0) return false;

    // 检查芯片 ID (BMP280 为 0x58, BME280 为 0x60)
    uint8_t id;
    read_registers(0xD0, &id, 1);
    if (id != 0x58 && id != 0x60) {
        std::cerr << "未找到 BMP280/BME280 芯片! ID: " << std::hex << (int)id << std::endl;
        return false;
    }

    read_calibration_data();

    // 配置寄存器: 过采样 (温度 x1, 气压 x4), 工作模式 (Normal)
    write_register(0xF4, 0x27);
    // 配置寄存器: 待机时间 0.5ms, IIR 滤波系数 16
    write_register(0xF5, 0x10);

    return true;
}

void BMP280::set_reference_pressure(float ref_pressure) {
    reference_pressure = ref_pressure;
}

bool BMP280::read_sensor(float& temperature, float& pressure, float& altitude) {
    uint8_t data[6];
    if (!read_registers(0xF7, data, 6)) return false;

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // --- 博世官方补偿算法 (温度) ---
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) >> 8;
    temperature /= 100.0f;

    // --- 博世官方补偿算法 (气压) ---
    int64_t p_var1, p_var2, p;
    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;
    
    if (p_var1 == 0) return false; // 避免除零
    
    p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
    pressure = (float)p / 256.0f; // 最终气压，单位 Pa

    // --- 国际标准大气压高度公式 ---
    // Altitude = 44330 * (1.0 - (p / p0) ^ (1/5.255))
    altitude = 44330.0f * (1.0f - pow(pressure / reference_pressure, 0.1903f));

    return true;
}
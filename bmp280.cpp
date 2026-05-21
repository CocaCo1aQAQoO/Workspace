#include "bmp280.h"
#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define BMP280_ADDR 0x76 // BMP280 常见的 I2C 地址，也可能是 0x77

BMP280::BMP280() : i2c_fd(-1), base_pressure(101325.0f) {}

BMP280::~BMP280() {
    if (i2c_fd >= 0) close(i2c_fd);
}

bool BMP280::write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(i2c_fd, buf, 2) == 2;
}

bool BMP280::read_registers(uint8_t reg, uint8_t* buffer, int length) {
    if (write(i2c_fd, &reg, 1) != 1) return false;
    return read(i2c_fd, buffer, length) == length;
}

void BMP280::read_calibration_data() {
    uint8_t calib[24];
    read_registers(0x88, calib, 24); // 0x88 是校准数据起始地址

    // 将连续的字节拼接成对应的整数校准参数
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

bool BMP280::init(const char* i2c_device_node) {
    i2c_fd = open(i2c_device_node, O_RDWR);
    if (i2c_fd < 0) return false;

    if (ioctl(i2c_fd, I2C_SLAVE, BMP280_ADDR) < 0) return false;

    // 检查芯片 ID (BMP280 的 ID 寄存器 0xD0 应该返回 0x58)
    uint8_t chip_id;
    if (!read_registers(0xD0, &chip_id, 1) || chip_id != 0x58) {
        std::cerr << "未找到 BMP280 传感器！" << std::endl;
        return false;
    }

    read_calibration_data();

    // 配置测量控制寄存器 0xF4:
    // osrs_t (温度过采样) = 1, osrs_p (气压过采样) = 4, mode (电源模式) = 3 (Normal)
    write_register(0xF4, 0x27);

    // 等待传感器稳定并进行第一次测量，将当前气压记录为 0 米地面的基准气压
    usleep(100000); 
    float temp;
    read_sensor_data(temp, base_pressure); 
    
    return true;
}

void BMP280::read_sensor_data(float& temperature, float& pressure) {
    uint8_t data[6];
    if (!read_registers(0xF7, data, 6)) return;

    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // --- 博世官方温度补偿算法 ---
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) >> 8;
    temperature /= 100.0f;

    // --- 博世官方气压补偿算法 ---
    int64_t p;
    int64_t p_var1, p_var2;
    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;

    if (p_var1 == 0) {
        pressure = 0; // 避免除以零
        return;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
    
    pressure = (float)p / 256.0f; // 最终绝对气压 (帕斯卡)
}

float BMP280::get_relative_altitude() {
    float temp, current_pressure;
    read_sensor_data(temp, current_pressure);
    
    // 气压高度公式推导
    float altitude = 44330.0f * (1.0f - pow(current_pressure / base_pressure, 1.0f / 5.255f));
    return altitude;
}
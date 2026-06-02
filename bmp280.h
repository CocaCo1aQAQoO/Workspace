#ifndef BMP280_H
#define BMP280_H

#include <string>
#include <cstdint>

class BMP280 {
public:
    // 默认传入 i2c-3 总线和 0x76 地址
    BMP280(const std::string& bus, int address = 0x76);
    ~BMP280();

    bool init();
    
    // 核心读取函数：返回温度(摄氏度)、气压(Pa)和相对海拔高度(米)
    bool read_sensor(float& temperature, float& pressure, float& altitude);
    
    // 设置初始地面气压，用于将当前高度归零
    void set_reference_pressure(float ref_pressure);

private:
    int fd;
    std::string bus_name;
    int device_address;
    float reference_pressure; // 初始地面气压
    
    // 芯片出厂校准参数
    uint16_t dig_T1; 
    int16_t dig_T2, dig_T3;
    
    // 🌟 已修复：去掉了非法的逗号和多余的类型声明
    uint16_t dig_P1; 
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    
    int32_t t_fine;

    bool write_register(uint8_t reg, uint8_t value);
    bool read_registers(uint8_t reg, uint8_t* data, int length);
    void read_calibration_data();
};

#endif
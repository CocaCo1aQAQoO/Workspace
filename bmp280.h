#ifndef BMP280_H
#define BMP280_H

#include <cstdint>

class BMP280 {
private:
    int i2c_fd;
    float base_pressure; // 起飞时的基准气压 (用于计算相对高度)

    // 存储从芯片 ROM 读出的厂家校准参数
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
    int16_t dig_P4;  int16_t dig_P5; int16_t dig_P6;
    int16_t dig_P7;  int16_t dig_P8; int16_t dig_P9;
    int32_t t_fine;

    // 底层 I2C 读写辅助函数
    bool write_register(uint8_t reg, uint8_t value);
    bool read_registers(uint8_t reg, uint8_t* buffer, int length);
    void read_calibration_data();

public:
    BMP280();
    ~BMP280();

    // 初始化 I2C 并读取校准数据
    bool init(const char* i2c_device_node);

    // 读取当前绝对气压 (Pa) 和温度 (摄氏度)
    void read_sensor_data(float& temperature, float& pressure);

    // 获取相对于起飞点的相对高度 (米)
    float get_relative_altitude();
};

#endif
#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <string>

class DataLogger {
private:
    std::ofstream log_file;
    int write_counter;

public:
    DataLogger();
    ~DataLogger();

    // 初始化日志文件，并写入 CSV 表头
    bool init(const std::string& filename);

    // 记录一帧飞行数据
    void log_frame(float time_sec, float pitch, float roll, float yaw, float alt, float m1, float m2, float m3, float m4);
};

#endif
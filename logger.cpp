#include "logger.h"
#include <iostream>

DataLogger::DataLogger() : write_counter(0) {}

DataLogger::~DataLogger() {
    if (log_file.is_open()) {
        log_file.flush();
        log_file.close();
    }
}

bool DataLogger::init(const std::string& filename) {
    // 以追加模式(app)打开文件，防止重启覆盖之前的记录
    log_file.open(filename, std::ios::out | std::ios::app);
    if (!log_file.is_open()) {
        return false;
    }
    
    // 写入标准的 CSV 表头
    log_file << "Time(s),Pitch,Roll,Yaw,Altitude(m),Motor1,Motor2,Motor3,Motor4\n";
    return true;
}

void DataLogger::log_frame(float time_sec, float pitch, float roll, float yaw, float alt, float m1, float m2, float m3, float m4) {
    if (!log_file.is_open()) return;

    // 写入数据（此时数据只存在于 Linux 的内存缓冲区，速度极快，不阻塞程序）
    log_file << time_sec << "," 
             << pitch << "," << roll << "," << yaw << "," 
             << alt << ","
             << m1 << "," << m2 << "," << m3 << "," << m4 << "\n";

    write_counter++;

    // 核心优化：每攒够 50 条数据 (约 0.5 秒)，强制刷入 SD 卡物理扇区一次
    // 这样既保证了 100Hz 循环的丝滑，又能在无人机意外断电时最大限度保留黑匣子数据
    if (write_counter >= 50) {
        log_file.flush();
        write_counter = 0;
    }
}
#include "imu.h"
#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// MPU6050 常用寄存器地址
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

IMUSensor::IMUSensor() {
    pitch = 0.0f; roll = 0.0f; yaw = 0.0f;
    // 这些零偏数据必须等你的硬件到了之后，平放测出来再填进去！
    gyro_x_offset = 0.0f; 
    gyro_y_offset = 0.0f; 
    gyro_z_offset = 0.0f;
}

IMUSensor::~IMUSensor() {
    if (i2c_fd >= 0) close(i2c_fd);
}

bool IMUSensor::init(const char* i2c_device_node) {
    // 1. 打开 Linux I2C 设备节点 (未来真实测试时传入 "/dev/i2c-1" 等)
    i2c_fd = open(i2c_device_node, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "打开 I2C 设备失败!" << std::endl;
        return false;
    }

    // 2. 绑定 MPU6050 的从机地址
    if (ioctl(i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        std::cerr << "绑定 MPU6050 地址失败!" << std::endl;
        return false;
    }

    // 3. 唤醒传感器 (向电源管理寄存器写 0)
    char buf[2] = {PWR_MGMT_1, 0x00};
    if (write(i2c_fd, buf, 2) != 2) {
        std::cerr << "唤醒 MPU6050 失败!" << std::endl;
        return false;
    }
    return true;
}

short IMUSensor::read_raw_data(int addr) {
    char reg[1] = {(char)addr};
    write(i2c_fd, reg, 1);
    char data[2] = {0};
    read(i2c_fd, data, 2);
    return (data[0] << 8) | data[1];
}

void IMUSensor::update(float dt) {
    // 1. 读取原始加速度计 (定姿态，长周期准确，有高频震动噪点)
    float acc_x = read_raw_data(ACCEL_XOUT_H) / 16384.0f; // 默认量程 +-2g
    float acc_y = read_raw_data(ACCEL_XOUT_H + 2) / 16384.0f;
    float acc_z = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0f;

    // 2. 读取原始陀螺仪 (防抖动，短周期准确，有低频零点漂移)
    float gyro_x = (read_raw_data(GYRO_XOUT_H) / 131.0f) - gyro_x_offset; 
    float gyro_y = (read_raw_data(GYRO_XOUT_H + 2) / 131.0f) - gyro_y_offset;
    // float gyro_z = (read_raw_data(GYRO_XOUT_H + 4) / 131.0f) - gyro_z_offset;

    // 3. 利用三角函数从加速度推导绝对角度 (单位：度)
    float acc_roll  = atan2(acc_y, acc_z) * 180.0 / M_PI;
    float acc_pitch = atan2(-acc_x, sqrt(acc_y*acc_y + acc_z*acc_z)) * 180.0 / M_PI;

    // 4. 👇 【核心升级】卡尔曼滤波数据融合
    // 传入：加速度计观测角度、陀螺仪角速度、时间差
    roll  = kalman_roll.get_angle(acc_roll, gyro_x, dt);
    pitch = kalman_pitch.get_angle(acc_pitch, gyro_y, dt);
    
    // 注意：偏航角 (Yaw) 通常无法通过加速度计补偿（因为重力垂直于 Z 轴），只能靠陀螺仪硬积分，或引入磁力计
    // yaw = yaw + gyro_z * dt; 
}
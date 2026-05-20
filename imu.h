#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

class IMUSensor {
private:
    int i2c_fd;             // I2C 设备文件描述符
    float gyro_x_offset;    // 陀螺仪 X 轴零偏补偿 (硬件到了需校准)
    float gyro_y_offset;
    float gyro_z_offset;

    // 互补滤波的参数 (通常取 0.98 左右)
    float alpha;            

    // 内部寄存器读取函数
    short read_raw_data(int addr);

public:
    // 当前解算出的姿态角
    float pitch;
    float roll;
    float yaw;

    IMUSensor();
    ~IMUSensor();

    // 初始化 I2C 连接并唤醒 MPU6050
    bool init(const char* i2c_device_node);

    // 核心：读取原始数据并执行互补滤波
    void update(float dt);
};

#endif
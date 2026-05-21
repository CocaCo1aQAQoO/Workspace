#ifndef KALMAN_H
#define KALMAN_H

class KalmanFilter {
private:
    // 卡尔曼核心状态变量
    float angle; // 滤波器计算出的最优角度
    float bias;  // 陀螺仪计算出的零偏漂移量
    float rate;  // 无偏角速度

    // 2x2 误差协方差矩阵 P (平铺为 4 个标量，极大提升运算速度)
    float P[2][2];

    // 卡尔曼调参核心
    float Q_angle; // 加速度计的过程噪声协方差
    float Q_bias;  // 陀螺仪漂移的过程噪声协方差
    float R_measure; // 加速度计的测量噪声协方差

public:
    KalmanFilter();

    // 核心算法：输入加速度计算出的角度、陀螺仪算出的角速度、以及时间差 dt
    float get_angle(float new_angle, float new_rate, float dt);
    
    // 设置初始参数
    void set_angle(float angle);
    void set_Q_angle(float Q_angle);
    void set_Q_bias(float Q_bias);
    void set_R_measure(float R_measure);
};

#endif
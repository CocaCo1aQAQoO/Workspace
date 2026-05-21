#include "kalman.h"

KalmanFilter::KalmanFilter() {
    // 初始化卡尔曼参数 (这三个值是无人机最常用的黄金初始值)
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f;
    bias = 0.0f;
    rate = 0.0f;

    // 初始化协方差矩阵
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
}

float KalmanFilter::get_angle(float new_angle, float new_rate, float dt) {
    // ================== 1. 预测阶段 (Predict) ==================
    // 根据先验物理模型预测当前状态
    rate = new_rate - bias;
    angle += dt * rate;

    // 更新误差协方差矩阵
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // ================== 2. 更新阶段 (Update) ==================
    // 计算卡尔曼增益 K (Kalman Gain)
    float S = P[0][0] + R_measure; // 估算误差
    float K[2]; // 2x1 增益向量
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 计算实际测量值与预测值的残差
    float y = new_angle - angle;

    // 结合卡尔曼增益，更新出最优估计角度和陀螺仪漂移
    angle += K[0] * y;
    bias += K[1] * y;

    // 更新误差协方差矩阵 P，为下一次循环做准备
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle; // 返回纯净、无延迟的最优角度
}

void KalmanFilter::set_angle(float new_angle) { angle = new_angle; }
void KalmanFilter::set_Q_angle(float q) { Q_angle = q; }
void KalmanFilter::set_Q_bias(float q) { Q_bias = q; }
void KalmanFilter::set_R_measure(float r) { R_measure = r; }
#include "pid.h"

PIDController::PIDController(float p, float i, float d, float max_i) {
    kp = p;
    ki = i;
    kd = d;
    max_integral = max_i;
    integral = 0.0f;
    prev_error = 0.0f;
}

void PIDController::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
}

float PIDController::update(float setpoint, float measured_value, float dt) {
    // 1. 计算当前误差 (Error)
    float error = setpoint - measured_value;

    // 2. 计算比例项 (P)
    float p_out = kp * error;

    // 3. 计算积分项 (I) 
    integral += error * dt;
    // 【工程关键】防积分饱和限幅
    if (integral > max_integral) {
        integral = max_integral;
    } else if (integral < -max_integral) {
        integral = -max_integral;
    }
    float i_out = ki * integral;

    // 4. 计算微分项 (D) - 注意防止 dt 为 0 导致崩溃
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - prev_error) / dt;
    }
    float d_out = kd * derivative;

    // 5. 记录本次误差，供下一次微分计算使用
    prev_error = error;

    // 6. 综合输出
    return p_out + i_out + d_out;
}
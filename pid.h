#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    float kp;             // 比例系数 (Proportional)
    float ki;             // 积分系数 (Integral)
    float kd;             // 微分系数 (Derivative)

    float integral;       // 误差累计值
    float prev_error;     // 上一次循环的误差
    float max_integral;   // 积分限幅阈值 (防积分饱和)

public:
    // 构造函数，初始化参数
    PIDController(float p, float i, float d, float max_i);

    // 重置 PID 状态（例如重新起飞时调用）
    void reset();

    // 核心更新函数：传入目标值、当前测量值和时间差，返回控制输出量
    float update(float setpoint, float measured_value, float dt);
};

#endif // PID_CONTROLLER_H
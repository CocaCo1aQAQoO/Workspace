#ifndef RC_MATH_H
#define RC_MATH_H

class RCMath {
public:
    // 处理死区
    // input: 原始输入 (-1.0 到 1.0)
    // deadband: 死区阈值 (例如 0.05 代表 5% 的死区)
    static float apply_deadband(float input, float deadband);

    // 处理 EXPO 指数曲线
    // input: 去除死区后的输入 (-1.0 到 1.0)
    // expo: 曲线弯曲程度 (0.0 是纯直线, 1.0 是纯立方曲线)
    static float apply_expo(float input, float expo);

    // 综合处理 (先死区，后曲线)
    static float process_channel(float input, float deadband, float expo);
};

#endif
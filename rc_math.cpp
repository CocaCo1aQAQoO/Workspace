#include "rc_math.h"
#include <cmath>

float RCMath::apply_deadband(float input, float deadband) {
    // 1. 如果在死区范围内，强制输出 0
    if (std::abs(input) <= deadband) {
        return 0.0f;
    }
    
    // 2. 如果超出了死区，我们需要把剩余的区间重新映射到 0~1.0
    // 防止摇杆一出死区，输出值突然产生跳变
    float sign = (input > 0.0f) ? 1.0f : -1.0f;
    return sign * ((std::abs(input) - deadband) / (1.0f - deadband));
}

float RCMath::apply_expo(float input, float expo) {
    // Expo 映射公式：混合了纯线性 (x) 和纯立方 (x^3)
    // 这样在输入为 0 附近时变化极小，在输入接近 1.0 或 -1.0 时迅速增大
    return expo * (input * input * input) + (1.0f - expo) * input;
}

float RCMath::process_channel(float input, float deadband, float expo) {
    // 限制原始输入在合法范围内
    if (input > 1.0f) input = 1.0f;
    if (input < -1.0f) input = -1.0f;

    // 先削平死区，再进行曲线弯折
    float deadband_result = apply_deadband(input, deadband);
    return apply_expo(deadband_result, expo);
}
#include <iostream>
#include <iomanip>
#include "pid.h"

// 模拟无人机物理响应的简单函数（替代真实环境中的电机和传感器）
float simulate_drone_physics(float current_angle, float control_output, float dt) {
    // 假设控制输出(类似电机转速差)直接转化为角加速度，并受到一定的物理阻尼影响
    float angular_velocity = control_output * 0.5f; 
    return current_angle + (angular_velocity * dt);
}

int main() {
    std::cout << "========== 无人机 PID 核心算法地面站仿真测试 ==========" << std::endl;

    // 实例化一个俯仰角(Pitch)的 PID 控制器
    // 参数设置: Kp=2.5, Ki=0.1, Kd=0.5, 积分限幅=50.0
    PIDController pitch_pid(2.5f, 0.1f, 0.5f, 50.0f);

    float current_pitch = 0.0f;   // 传感器读取：当前飞机是水平的 (0度)
    float target_pitch = 30.0f;  // 遥控器输入：目标是仰起 30度
    float dt = 0.01f;             // 系统频率 100Hz，每次循环 10 毫秒

    std::cout << "初始状态 -> 飞机角度: 0度  |  推杆目标: 30度\n" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    // 模拟 100 次高频控制循环 (总计真实时间 1 秒)
    for (int i = 1; i <= 100; i++) {
        // 步骤 A: PID 根据误差计算出应当给电机的调整量
        float control_output = pitch_pid.update(target_pitch, current_pitch, dt);

        // 步骤 B: 物理系统响应（真实情况是电机提速改变了机身角度）
        current_pitch = simulate_drone_physics(current_pitch, control_output, dt);

        // 步骤 C: 打印遥测数据（每 10 次循环打印一次）
        if (i % 10 == 0) { 
            std::cout << "循环 第 " << std::setw(3) << i << " 次 (T=" << i*dt << "s) | "
                      << "PID 输出拉力: " << std::setw(6) << control_output << " | "
                      << "当前飞机角度: " << current_pitch << " °" << std::endl;
        }
    }

    std::cout << "\n========== 仿真结束：飞机姿态已锁定 ==========" << std::endl;
    return 0;
}
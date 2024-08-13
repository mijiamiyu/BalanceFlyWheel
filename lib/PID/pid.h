#ifndef __PID_H
#define __PID_H

#include <limits>

// 没有最大最小值限制时候的赋值
#define NO_MAX_LIMIT std::numeric_limits<double>::max()
#define NO_MIN_LIMIT std::numeric_limits<double>::lowest()

// PID 参数
typedef struct PIDParameters {
    double Kp;                    // 比例系数
    double Ki;                    // 积分系数
    double Kd;                    // 微分系数
    double outMax = NO_MAX_LIMIT; // 输出最大值, 默认没有最大限制
    double outMin = NO_MIN_LIMIT; // 输出最小值
    PIDParameters(double p, double i, double d, double outMax = NO_MAX_LIMIT, double outMin = NO_MIN_LIMIT) : Kp(p), Ki(i), Kd(d), outMax(outMax), outMin(outMin) {}
} PIDParameters;

// 控制模式
typedef enum _ControlMode {
    Incremental, // 增量 PID
    Positional   // 位置 PID
} ControlMode;

class PIDController {
  public:
    PIDParameters params;
    ControlMode mode;
    volatile double TargetValue; // 目标点
    double lastError;            // 上一次误差
    double lastlastError;        // 上上次误差
    double integral;             // 积分项累计
    double output;               // 控制输出
    double error;
    double potentialOutput; // 潜在的未限制输出
    double delta;
    double potentialIntegral;

    PIDController(const PIDParameters& parameters, ControlMode controlMode);
    void setTargetValue(double TargetValue);
    double getOutput() const;
    double update(double actualValue);
};

#endif
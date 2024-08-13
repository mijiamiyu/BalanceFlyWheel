#include "pid.h"
#include <Arduino.h>
/**
 * @brief       Construct a new PIDController::PIDController object
 *
 * @param       parameters pid参数
 * @param       controlMode 控制模式，选择为增量PID或者位置式PID
 */
PIDController::PIDController(const PIDParameters& parameters, ControlMode controlMode)
    : params(parameters), mode(controlMode), TargetValue(0.0), lastError(0.0),
      lastlastError(0.0), integral(0.0), output(0.0), potentialOutput(0.0), potentialIntegral(0.0) {}

/**
 * @brief 设置PID控制器的设定点
 *
 * @param sp 新的设定点值
 */
void PIDController::setTargetValue(double TargetValue) {
    this->TargetValue = TargetValue;
}

/**
 * @brief 获取当前的控制输出值
 *
 * @return double 当前的控制输出
 */
double PIDController::getOutput() const {
    return output;
}

/**
 * @brief 根据实际测量值更新PID控制器并计算新的输出
 *
 * @param actualValue 实际测量值
 * @return double 新计算出的控制输出
 */
double PIDController::update(double actualValue) {
    error = TargetValue - actualValue;

    switch (mode) {
        case Incremental:
            delta = params.Kp * (error - lastError) +
                    params.Ki * error +
                    params.Kd * (error - 2 * lastError + lastlastError);

            lastlastError = lastError;
            lastError = error;

            return delta;

        // 如果情况都不符合，默认为Positional
        default:

        case Positional:
            potentialIntegral = integral + error;
            potentialOutput = params.Kp * error + params.Ki * potentialIntegral + params.Kd * (error - lastError);
            // 检查输出是否会饱和并决定是否累加积分
            if ((potentialOutput < params.outMax) && (potentialOutput > params.outMin)) {
                integral = potentialIntegral; // 更新积分项
            }
            // 应用输出限制
            output = potentialOutput;
            if (output > params.outMax)
                output = params.outMax;
            else if (output < params.outMin)
                output = params.outMin;

            lastlastError = lastError;
            lastError = error;

            return output;
    }
}

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include <cstdint>

#define MOTOR_NUM 4                                // 电机的数量
#define RPS_TO_RADPS(rps) ((rps) * 2 * M_PI)       // 每秒转速转换成每秒角速度
#define RADPS_TO_RPS(radps) ((radps) / (2 * M_PI)) // 每秒角速度转换成每秒转速

// 定义宏，初始化电机数组
#define INIT_MOTORS(motors, size)       \
    for (size_t i = 0; i < size; i++) { \
        motors[i] = nullptr;            \
    }

// 用户定义的字面量处理kg·cm转换为Nm（牛顿·米）
constexpr double operator"" _kgcm(long double val) {
    return static_cast<double>(val) * 0.0981; // 1 kg·cm = 0.0981 N·m
}

// 用户定义的字面量处理kg·mm转换为kg·m²（公斤·米²）
constexpr double operator"" _kgmm2(long double val) {
    return static_cast<double>(val) * 1e-6; // 1 kg·mm² = 1e-6 kg·m²
}

// 标准单位字面量，不进行转换
constexpr double operator"" _Nm(long double val) {
    return static_cast<double>(val); // 直接返回相同值
}

constexpr double operator"" _kgm2(long double val) {
    return static_cast<double>(val); // 直接返回相同值
}

class Motor {
  private:
    const uint32_t CLK = 80000000;
    uint8_t chan;
    uint32_t freq;
    uint8_t res;
    uint8_t pwm;
    uint8_t dir_pin;
    uint8_t A;
    uint8_t B;

    void setDir(int dir);
    static void handleAInterrupt(void* arg); // 编码器A相的中断函数
    static void handleBInterrupt(void* arg); // 编码器B相的中断函数

  public:
    ControlMode controlMode;
    uint32_t max_duty;
    int32_t duty;                 // 记录当前的duty值
    volatile int dir;             // 转的方向，1为顺时针转动，0为不动，-1为逆时针转动，此值是通过编码器得到的值，而不是通过设置得到的
    volatile int32_t encoder_cnt; // 当前的编码计数值, 顺时针转动增加，逆时针减小
    volatile double angVel;       // 当前的角速度
    volatile int lastA;           // 上一次A相的读值
    volatile int lastB;           // 上一次B相的读值
    double reductionRatio;        // 电机减速比
    uint32_t PPR;                 // 编码器线数
    PIDController* pid;
    double Torque;      // 最大电机扭矩
    double Moi;         // 电机飞轮的转动惯量
    double max_AA;      // 最大角加速度
    volatile double AA; // 角加速度

    Motor(uint8_t chan, uint32_t freq, uint8_t pwm, uint8_t dir, uint8_t A, uint8_t B, double reductionRatio, uint32_t PPR, double Torque, double Moi);
    ~Motor();
    void begin(double Kp = 0, double Ki = 0, double Kd = 0, ControlMode controlMode = Incremental);
    void setDuty(int32_t duty);
    void stop();
    void incDuty(int32_t delta_duty);
    void setSpeed(double angVel);
    void setAA(double AA);
};

extern Motor* motors[MOTOR_NUM];

void getSpeedTimerStart(double interval_ms);

void setSpeedPIDStart(double interval_ms);

void setAATimerStart(double interval_ms);
#endif
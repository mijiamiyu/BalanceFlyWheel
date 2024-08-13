#include "Motor.h"
#include "esp32-config.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal-ledc.h"
#include <math.h>
#include <unistd.h>

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))
#define SPEED_PID_TASK_SIZE 2048

Motor* motors[MOTOR_NUM];     // 定义电机结构体
double get_speed_interval_ms; // 定义一个变量存储计算速度定时器的时间间隔，单位为毫秒
double set_speed_interval_s;  // 定义一个变量存储根据加速度设置速度的定时器的时间间隔，单位为秒

/**
 * @brief       Construct a new Motor:: Motor object
 *
 * @param       chan 设置的PWM对应的通道数
 * @param       freq 设置pwm波的频率
 * @param       pwm pwm连接的引脚
 * @param       dir 控制电机方向的数字引脚， 默认低电平为正转，高电平反转
 * @param       A 编码器A相连接的引脚
 * @param       B 编码器B相连接的引脚
 */
Motor::Motor(uint8_t chan, uint32_t freq, uint8_t pwm, uint8_t dir, uint8_t A, uint8_t B, double reductionRatio, uint32_t PPR, double Torque, double Moi)
    : chan(chan), freq(freq), pwm(pwm), dir_pin(dir), A(A), B(B), reductionRatio(reductionRatio), PPR(PPR), encoder_cnt(0), dir(0), angVel(0.0), Torque(Torque), Moi(Moi), AA(0.0) {
}

/**
 * @brief       Destroy the Motor:: Motor object
 *
 */
Motor::~Motor() {
    ledcDetachPin(pwm);
}

/**
 * @brief       初始化Motor
 *
 */
void Motor::begin(double Kp, double Ki, double Kd, ControlMode controlMode) {
    // 计算最大分辨率
    uint8_t temp = (uint8_t)(log2((double)CLK / (double)freq));
    res = max(min(temp, SOC_LEDC_TIMER_BIT_WIDE_NUM), 1);

    max_duty = (1 << res) - 1;

    if (Torque != 0 && Moi != 0)
        max_AA = Torque / Moi;
    else
        max_AA = 0;

    // 设置pwm
    ledcSetup(chan, freq, res);
    ledcAttachPin(pwm, chan);
    // 设置方向引脚为数字输出
    pinMode(dir_pin, OUTPUT);

    // 设置编码器的外部中断
    pinMode(A, INPUT_PULLUP);
    pinMode(B, INPUT_PULLUP);

    lastA = digitalRead(A);
    lastB = digitalRead(B);

    attachInterruptArg(A, handleAInterrupt, this, CHANGE);
    attachInterruptArg(B, handleBInterrupt, this, CHANGE);

    // 初始化pid控制器
    pid = new PIDController(PIDParameters(Kp, Ki, Kd, max_duty, -((double)max_duty)), controlMode);
    this->controlMode = controlMode;
}

/**
 * @brief       通过设置pwm的占空比去设置电机的速度，输入值范围是[-max_duty, max_duty], 输入值绝对值越大，转速越大，符号代表正反转
 *              此电机占空比绝对值越小速度越大, 所以需要将输入值进行转换
 *
 * @param       duty 占空比值
 */
void Motor::setDuty(int32_t duty) {
    // 设置方向
    int dir = duty >= 0 ? 1 : -1;
    setDir(dir);

    // 设置转速
    uint32_t abs_duty = (uint32_t)(duty >= 0 ? duty : -duty); // 记录占空比的绝对值
    this->duty = abs_duty > max_duty ? dir * max_duty : duty; // 记录占空比值
    ledcWrite(chan, max_duty - abs_duty);
}

/**
 * @brief       设置方向，1为正转顺时针，-1为反转逆时针
 *
 * @param       dir
 */
void Motor::setDir(int dir) {
    // 设置前正转，要设置成反转 或者 设置前反转，要设置成正转
    // 要先停下来，然后才能设置方向，防止突然反转让电机烧坏
    // if ((duty >= 0 && dir == -1) || (duty < 0 && dir == 1)) {
    //     stop();
    //     // 直到当前的速度为0才可以变换方向
    //     while (angVel != 0)
    //         ;
    // }
    uint8_t val = dir == 1 ? HIGH : LOW; // 设置电平
    digitalWrite(this->dir_pin, val);
}

/**
 * @brief       将电机停止
 *
 */
void Motor::stop() {
    ledcWrite(chan, max_duty);
}

/**
 * @brief       根据现在的duty值增加delta_duty值，去改变转速
 *
 * @param       delta_duty 改变的duty值，可为负数，负数为减少
 */
void Motor::incDuty(int32_t delta_duty) {
    setDuty(this->duty + delta_duty);
}

/**
 * @brief       编码器A相的中断函数
 *
 * @param       arg 强制转换为Motor类型，以此访问内部元素
 */
void IRAM_ATTR Motor::handleAInterrupt(void* arg) {
    Motor* motor = static_cast<Motor*>(arg);
    int currentA = digitalRead(motor->A); // 读取A相当前状态
    int currentB = digitalRead(motor->B); // 读取B相当前状态

    // 判断A相的变化
    if (currentA != motor->lastA) {
        // 根据A和B相的状态决定计数方向
        if (currentA == currentB) {
            motor->encoder_cnt--; // 顺时针
        } else {
            motor->encoder_cnt++; // 逆时针
        }
    }
    motor->lastA = currentA; // 更新A相的状态
}

/**
 * @brief       编码器B相的中断函数
 *
 * @param       arg 强制转换为Motor类型，以此访问内部元素
 */
void IRAM_ATTR Motor::handleBInterrupt(void* arg) {
    Motor* motor = static_cast<Motor*>(arg);
    int currentA = digitalRead(motor->A); // 读取A相当前状态
    int currentB = digitalRead(motor->B); // 读取B相当前状态

    // 判断B相的变化
    if (currentB != motor->lastB) {
        // 根据A和B相的状态决定计数方向
        if (currentA == currentB) {
            motor->encoder_cnt++; // 逆时针
        } else {
            motor->encoder_cnt--; // 顺时针
        }
    }
    motor->lastB = currentB; // 更新B相的状态
}

/**
 * @brief       设置目标角速度，通过PID趋近目标角速度
 *
 * @param       angVel
 */
void Motor::setSpeed(double angVel) {
    pid->setTargetValue(angVel);
}

void Motor::setAA(double AA) {
    this->AA = max(min(AA, max_AA), -max_AA);
}

/**
 * @brief       此定时器计算每一个电机实时速度
 *
 */
void getSpeedTimer() {
    // 遍历每个 Motor
    for (int i = 0; i < MOTOR_NUM; i++) {
        Motor* motor = motors[i];
        if (motor != nullptr) {
            // 计算角速度 (角度变化量 / 时间间隔)
            // 四倍频编码器，所以计算时需要考虑四倍频
            double deltaEncoderCount = static_cast<double>(motor->encoder_cnt);                                         // 当前计数差，因为上一次清零
            double interval_s = get_speed_interval_ms / 1000.0;                                                         // 转换毫秒到秒
            double revolutions = (deltaEncoderCount / 4.0) / (static_cast<double>(motor->PPR) * motor->reductionRatio); // 计算转过的圈数

            // 考虑减速比，计算输出轴的角速度
            motor->angVel = (revolutions * 2 * M_PI) / interval_s;

            // 根据角速度设置方向 dir
            if (motor->angVel > 0) {
                motor->dir = 1; // 顺时针
            } else if (motor->angVel < 0) {
                motor->dir = -1; // 逆时针
            } else {
                motor->dir = 0; // 停止
            }

            // 清零 encoder_cnt
            motor->encoder_cnt = 0;
        }
    }
}

/**
 * @brief       根据输入参数设定定时器预分频器，并运行定时器
 *
 * @param       interval_ms
 */
void getSpeedTimerStart(double interval_ms) {
    // 全局变量存储测量间隔
    get_speed_interval_ms = interval_ms;

    // 系统时钟频率（假设为 80 MHz）
    const double SYSTEM_CLOCK_FREQ = 80e6;           // 80 MHz
    double timer_interval_us = interval_ms * 1000.0; // 将间隔转换为微秒

    // 确定预分频器值，以确保定时器频率适中，这里设定为使定时器频率约为1MHz
    uint32_t prescaler = 80;

    // 计算定时器计数值，正确地使用预分频器值
    double timer_count = (interval_ms / 1000.0) * (SYSTEM_CLOCK_FREQ / (prescaler + 1));

    // 初始化定时器
    hw_timer_t* timer = timerBegin(0, prescaler, true);  // 定时器0，预分频器
    timerAttachInterrupt(timer, &getSpeedTimer, true);   // 附加中断函数
    timerAlarmWrite(timer, (uint32_t)timer_count, true); // 设置定时器计数值
    timerAlarmEnable(timer);                             // 启用定时器
}

/**
 * @brief       根据PID更新占空比达到目标速度
 *
 * @param       arg
 */
void setSpeedPID(void* arg) {
    double interval_ms = *static_cast<double*>(arg); // 从 void* 转换回 double
    while (true) {
        for (int i = 0; i < MOTOR_NUM; i++) {
            Motor* motor = motors[i];
            if (motor != nullptr) {
                double output = motor->pid->update(motor->angVel); // 计算 PID 控制器的输出
                switch (motor->controlMode) {
                    case Incremental:
                        motor->incDuty(output); // 应用新的占空比
                        break;
                    case Positional:
                        motor->setDuty(output); // 应用新的占空比
                        break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(interval_ms)); // 延迟一段时间再次执行
    }
}

/**
 * @brief       启动 setSpeedPID 任务
 *
 * @param       interval_ms PID任务的间隔时间
 */
void setSpeedPIDStart(double interval_ms) {
    static double storedInterval = interval_ms;                                             // 存储延时参数
    xTaskCreate(setSpeedPID, "SetSpeedPID", SPEED_PID_TASK_SIZE, &storedInterval, 1, NULL); // 创建任务
}

// 定时器回调函数，用于调整电机速度
void setAATimer() {
    static double temp_speed = 0;
    for (int i = 0; i < MOTOR_NUM; i++) {
        Motor* motor = motors[i];
        if (motor != nullptr) {
            // 根据加速度和时间间隔计算下一个速度
            double nextSpeed;
            // if (motor->angVel == 0) {
            // 这个遍历是防止加速度值过小启动不起来电机
            temp_speed = temp_speed + motor->AA * set_speed_interval_s;
            nextSpeed = temp_speed;
            // } else {
            //     if (temp_speed != 0)
            //         temp_speed = 0;
            //     nextSpeed = motor->angVel + motor->AA * set_speed_interval_s;
            // }

            motor->setSpeed(nextSpeed); // 设置新的速度
        }
    }
}

// 启动定时器，设置定时器间隔
void setAATimerStart(double interval_ms) {
    set_speed_interval_s = interval_ms / 1000.0; // 存储以秒为单位的时间间隔

    // 系统时钟频率（假设为 80 MHz）
    const double SYSTEM_CLOCK_FREQ = 80e6;
    double timer_interval_us = interval_ms * 1000.0; // 转换为微秒

    // 确定预分频器值，以确保定时器频率适中，这里设定为使定时器频率约为1MHz
    uint32_t prescaler = 80;

    // 计算定时器计数值，正确地使用预分频器值
    double timer_count = (interval_ms / 1000.0) * (SYSTEM_CLOCK_FREQ / (prescaler + 1));

    // 初始化定时器
    hw_timer_t* timer = timerBegin(1, prescaler, true);  // 使用定时器1
    timerAttachInterrupt(timer, &setAATimer, true);      // 附加中断函数
    timerAlarmWrite(timer, (uint32_t)timer_count, true); // 设置定时器计数值
    timerAlarmEnable(timer);                             // 启用定时器
}

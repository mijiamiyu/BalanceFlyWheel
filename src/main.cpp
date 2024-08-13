#include "DATASCOPE.h"
#include "Motor.h"
#include "esp32-config.h"
#include <Arduino.h>

// 虚拟上位机上位机
DATASCOPE Scope;

// 电机定义
Motor motor0(0, 1000, P0, P1, P_Y, P_P, 1, 100, 0.7_kgcm, 154.78_kgmm2);

void test(void*);

void setup() {
    Serial.begin(128000);           // 串口使能,上位机接收波特率为128000
    INIT_MOTORS(motors, MOTOR_NUM); // 初始化电机组
    // 电机使能并传递给motors结构体，让定时器中断算实际角速度
    motor0.begin(5000, 100, 1000, Positional);
    motors[0] = &motor0;

    // 中断启动
    // 计算电机速度定时器开启
    getSpeedTimerStart(5);

    // 电机速度PID任务运行
    setSpeedPIDStart(1);

    // 电机加速度定时器开启
    setAATimerStart(5);

    xTaskCreate(test, "test", 2048, NULL, 1, NULL); // 创建测试任务
}

void loop() {
    Scope.Display(RADPS_TO_RPS(motors[0]->angVel), motors[0]->pid->error, motors[0]->pid->lastError, motors[0]->pid->delta, motors[0]->pid->potentialOutput);
    delay(10);
}

void test(void* arg) {
    while (true) {
        motor0.setAA(RPS_TO_RADPS(0.5));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(-0.5));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(1));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(-1));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(2));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(-2));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(2.5));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(-2.5));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(2.6));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(-2.6));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(10));
        delay(5000);
        motor0.setAA(RPS_TO_RADPS(-10));
        delay(5000);
        // motor0.setSpeed(RPS_TO_RADPS(10));
        // delay(5000);
        // motor0.setSpeed(RPS_TO_RADPS(0));
        // delay(5000);
        // motor0.setSpeed(RPS_TO_RADPS(-10));
        // delay(5000);
        // motor0.setSpeed(RPS_TO_RADPS(40));
        // delay(5000);
        // motor0.setSpeed(RPS_TO_RADPS(-40));
        // delay(5000);
        // motor0.setSpeed(RPS_TO_RADPS(60));
        // delay(5000);
    }
}
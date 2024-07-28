#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>

class Motor {
  public:
    Motor(uint8_t chan, uint32_t freq, uint8_t res);

  private:
    uint8_t chan;
    uint32_t freq;
    uint8_t res;
};

#endif
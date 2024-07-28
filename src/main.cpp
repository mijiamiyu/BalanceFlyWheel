#include "esp32_config.h"
#include <Arduino.h>


void setup() {
    ledcSetup(0, 50, 12);
    ledcAttachPin(P0, 0);
}

void loop() {
    ledcWrite(0, 102);
    delay(1000);
    ledcWrite(0, 512);
    delay(1000);
}

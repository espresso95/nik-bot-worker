#include <Arduino.h>
#include "rgb_led.h"

RgbLed rgb;

void setup() {
  rgb.begin();
}

void loop() {
  rgb.toggle();
  delay(1000);
}
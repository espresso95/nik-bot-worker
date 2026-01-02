#pragma once
#include <Arduino.h>
#include "pins.h"

class RgbLed {
 public:
  RgbLed() = default;

  void begin() {
    pinMode(Pins::kRgbRed, OUTPUT);
    pinMode(Pins::kRgbGreen, OUTPUT);
    pinMode(Pins::kRgbBlue, OUTPUT);
    off();
  }

  void setColor(bool red, bool green, bool blue) {
    digitalWrite(Pins::kRgbRed, red ? HIGH : LOW);
    digitalWrite(Pins::kRgbGreen, green ? HIGH : LOW);
    digitalWrite(Pins::kRgbBlue, blue ? HIGH : LOW);
  }

  void on() { setColor(true, true, true); }
  void off() { setColor(false, false, false); }

  void setRed() { setColor(true, false, false); }
  void setGreen() { setColor(false, true, false); }
  void setBlue() { setColor(false, false, true); }

  void toggle() {
    is_on_ = !is_on_;
    is_on_ ? on() : off();
  }

 private:
  bool is_on_ = false;
};

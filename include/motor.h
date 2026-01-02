#pragma once
#include <Arduino.h>
#include "pins.h"

// Single DC motor controlled by an H-bridge (2 pins)
class Motor {
 public:
  Motor(uint8_t pin_a, uint8_t pin_b, bool pwm_capable = false, bool inverted = false)
      : pin_a_(pin_a), pin_b_(pin_b), pwm_capable_(pwm_capable), inverted_(inverted) {}

  void begin() {
    pinMode(pin_a_, OUTPUT);
    pinMode(pin_b_, OUTPUT);
    stop();
  }

  void forward(uint8_t speed = 255) {
    if (inverted_) {
      reverseInternal(speed);
    } else {
      forwardInternal(speed);
    }
  }

  void reverse(uint8_t speed = 255) {
    if (inverted_) {
      forwardInternal(speed);
    } else {
      reverseInternal(speed);
    }
  }

  void stop() {
    digitalWrite(pin_a_, LOW);
    digitalWrite(pin_b_, LOW);
  }

 private:
  void forwardInternal(uint8_t speed) {
    if (pwm_capable_ && speed < 255) {
      analogWrite(pin_a_, speed);
      digitalWrite(pin_b_, LOW);
    } else {
      digitalWrite(pin_a_, HIGH);
      digitalWrite(pin_b_, LOW);
    }
  }

  void reverseInternal(uint8_t speed) {
    if (pwm_capable_ && speed < 255) {
      digitalWrite(pin_a_, LOW);
      analogWrite(pin_b_, speed);
    } else {
      digitalWrite(pin_a_, LOW);
      digitalWrite(pin_b_, HIGH);
    }
  }

  uint8_t pin_a_;
  uint8_t pin_b_;
  bool pwm_capable_;
  bool inverted_;
};

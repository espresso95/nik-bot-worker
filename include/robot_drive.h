#pragma once
#include <Arduino.h>
#include "motor.h"
#include "pins.h"

// Controls all 4 motors for robot drive system
class MotorController {
 public:
  RobotDrive()
      : motor1_(Pins::kMotorOutA1, Pins::kMotorOutB1, true, true),   // Left front, inverted
        motor2_(Pins::kMotorOutA2, Pins::kMotorOutB2, true, false),  // Right front
        motor3_(Pins::kMotorOutA3, Pins::kMotorOutB3, false, true),  // Left back, inverted
        motor4_(Pins::kMotorOutA4, Pins::kMotorOutB4, false, false) {} // Right back

  void begin() {
    motor1_.begin();
    motor2_.begin();
    motor3_.begin();
    motor4_.begin();
  }

  void forward(uint8_t speed = 255) {
    motor1_.forward(speed);
    motor2_.forward(speed);
    motor3_.forward(speed);
    motor4_.forward(speed);
  }

  void reverse(uint8_t speed = 255) {
    motor1_.reverse(speed);
    motor2_.reverse(speed);
    motor3_.reverse(speed);
    motor4_.reverse(speed);
  }

  void turnLeft(uint8_t speed = 255) {
    motor1_.reverse(speed);
    motor3_.reverse(speed);
    motor2_.forward(speed);
    motor4_.forward(speed);
  }

  void turnRight(uint8_t speed = 255) {
    motor1_.forward(speed);
    motor3_.forward(speed);
    motor2_.reverse(speed);
    motor4_.reverse(speed);
  }

  void stop() {
    motor1_.stop();
    motor2_.stop();
    motor3_.stop();
    motor4_.stop();
  }

  // Individual motor access if needed
  Motor& motor1() { return motor1_; }
  Motor& motor2() { return motor2_; }
  Motor& motor3() { return motor3_; }
  Motor& motor4() { return motor4_; }

 private:
  Motor motor1_;
  Motor motor2_;
  Motor motor3_;
  Motor motor4_;
};

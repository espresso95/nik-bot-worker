#pragma once
#include <Arduino.h>
#include "pins.h"

// HC-SR04 compatible ultrasonic sensor using single-pin mode (trig+echo tied together)
class UltrasonicSensor {
 public:
  UltrasonicSensor() = default;

  void begin() {
    pinMode(Pins::kUltrasonicTrigEcho, OUTPUT);
    digitalWrite(Pins::kUltrasonicTrigEcho, LOW);
  }

  // Returns distance in cm, or -1 on timeout
  long readDistanceCm(uint16_t timeout_us = 25000) {
    // Send trigger pulse (10μs HIGH)
    pinMode(Pins::kUltrasonicTrigEcho, OUTPUT);
    digitalWrite(Pins::kUltrasonicTrigEcho, LOW);
    delayMicroseconds(2);
    digitalWrite(Pins::kUltrasonicTrigEcho, HIGH);
    delayMicroseconds(10);
    digitalWrite(Pins::kUltrasonicTrigEcho, LOW);

    // Switch to input mode to read echo
    pinMode(Pins::kUltrasonicTrigEcho, INPUT);

    // Wait for echo pulse to start (HIGH)
    unsigned long start = micros();
    while (digitalRead(Pins::kUltrasonicTrigEcho) == LOW) {
      if (micros() - start > timeout_us) {
        return -1;  // Timeout
      }
    }

    // Measure echo pulse duration
    unsigned long pulseStart = micros();
    while (digitalRead(Pins::kUltrasonicTrigEcho) == HIGH) {
      if (micros() - pulseStart > timeout_us) {
        return -1;  // Timeout
      }
    }
    unsigned long pulseEnd = micros();
    unsigned long echoUs = pulseEnd - pulseStart;

    // Convert to cm: speed of sound = 343 m/s
    // Distance = (time * speed) / 2 (round trip)
    // 1 cm = 58 μs round-trip
    return (long)(echoUs / 58UL);
  }

  // Convenience method for checking if obstacle is within range
  bool isObstacleWithin(uint16_t distance_cm) {
    long dist = readDistanceCm();
    return (dist > 0 && dist <= distance_cm);
  }

 private:
  // No state needed - pin is defined in Pins namespace
};

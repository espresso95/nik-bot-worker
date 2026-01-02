// Ultrasonic one-pin demo (Trig+Echo on D10 as wired in Zeus kit)
#include <Arduino.h>
#include "../../include/pins.h"
#include "demos.h"

namespace ultrasonic_demo {
long readCm(uint16_t timeout_us = 25000) {
  const uint8_t pin = Pins::kUltrasonicTrigEcho;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);

  pinMode(pin, INPUT);
  unsigned long start = micros();
  while (digitalRead(pin) == LOW) if ((micros() - start) > timeout_us) return -1;

  unsigned long echoStart = micros();
  while (digitalRead(pin) == HIGH) if ((micros() - echoStart) > timeout_us) return -1;
  unsigned long echoUs = micros() - echoStart;

  return (long)(echoUs / 58UL); // cm
}
}

void ultrasonicDemoSetup() {
  Serial.begin(115200);
  Serial.println("Ultrasonic demo: send distance every 500 ms");
}

void ultrasonicDemoLoop() {
  long cm = ultrasonic_demo::readCm(20000);
  Serial.print("cm=");
  Serial.println(cm);
  delay(500);
}

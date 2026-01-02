// Grayscale array + obstacle sensors via 74HC165 shift register
#include <Arduino.h>
#include "../../include/pins.h"

uint16_t readBits(uint8_t nbits) {
  digitalWrite(Pins::kShiftLatch, LOW);
  delayMicroseconds(5);
  digitalWrite(Pins::kShiftLatch, HIGH);
  delayMicroseconds(5);

  uint16_t v = 0;
  for (uint8_t i = 0; i < nbits; i++) {
    v <<= 1;
    v |= digitalRead(Pins::kShiftData) ? 1 : 0;
    digitalWrite(Pins::kShiftClock, HIGH);
    delayMicroseconds(2);
    digitalWrite(Pins::kShiftClock, LOW);
    delayMicroseconds(2);
  }
  return v;
}

void setup() {
  pinMode(Pins::kShiftData, INPUT);
  pinMode(Pins::kShiftClock, OUTPUT);
  pinMode(Pins::kShiftLatch, OUTPUT);
  digitalWrite(Pins::kShiftClock, LOW);
  digitalWrite(Pins::kShiftLatch, HIGH);

  Serial.begin(115200);
  Serial.println("74HC165 demo: grayscale(8) + obstacle(2)");
}

void loop() {
  uint16_t v10 = readBits(10);
  uint8_t grayscale = (uint8_t)(v10 >> 2);
  uint8_t obstacle = (uint8_t)(v10 & 0x03);

  Serial.print("grayscale=");
  for (int i = 7; i >= 0; i--) Serial.print((grayscale >> i) & 1);
  Serial.print(" obstacle=");
  Serial.println(obstacle, BIN);
  delay(200);
}

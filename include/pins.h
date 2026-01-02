#pragma once
#include <Arduino.h>

// Arduino Uno R3 (SunFounder R3) pin map for Zeus Car Shield components.
namespace Pins {

// Built-in / always-connected peripherals
constexpr uint8_t kIrReceiver = 2;              // HS0038B signal

// Ultrasonic: Trig and Echo tied together on D10
constexpr uint8_t kUltrasonicTrigEcho = 10;

// 74HC165 chain for grayscale + obstacle avoidance (16-bit shift-in)
constexpr uint8_t kShiftData = 7;  // Q7 -> Arduino data in
constexpr uint8_t kShiftClock = 8; // CP
constexpr uint8_t kShiftLatch = 9; // PL (parallel load)

// RGB LED strips (two ports wired in parallel)
constexpr uint8_t kRgbBlue = 11;   // ~11
constexpr uint8_t kRgbRed = 12;
constexpr uint8_t kRgbGreen = 13;

// Motor driver (TC1508S x2 -> 4 motors)
constexpr uint8_t kMotorOutA1 = 3;   // ~3
constexpr uint8_t kMotorOutB1 = 4;
constexpr uint8_t kMotorOutA2 = 5;   // ~5
constexpr uint8_t kMotorOutB2 = 6;   // ~6
constexpr uint8_t kMotorOutA3 = A3;
constexpr uint8_t kMotorOutB3 = A2;
constexpr uint8_t kMotorOutA4 = A1;
constexpr uint8_t kMotorOutB4 = A0;

// Camera adapter / I2C / UART (ESP32-CAM + QMC6310)
constexpr uint8_t kUartRx = 0;   // D0
constexpr uint8_t kUartTx = 1;   // D1
constexpr uint8_t kI2cSda = A4;  // SDA
constexpr uint8_t kI2cScl = A5;  // SCL

}  // namespace Pins

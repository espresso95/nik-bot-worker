#include <Arduino.h>
#include "rgb_led.h"
#include "robot_drive.h"

RgbLed rgb;
MotorController motors;

void setup() {
  rgb.begin();
  motors.begin();
}

void loop() {
  rgb.setGreen();
  motors.forward(200);
  delay(2000);
  
  motors.stop();
  rgb.off();
  delay(500);
  
  rgb.setRed();
  motors.reverse(200);
  delay(2000);
  
  motors.stop();
  rgb.off();
  delay(500);
}
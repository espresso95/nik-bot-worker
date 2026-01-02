#include <Arduino.h>
#include "rgb_led.h"
#include "robot_drive.h"
#include "ultrasonic.h"

RgbLed rgb;
MotorController motors;
UltrasonicSensor ultrasonic;

void setup() {
  rgb.begin();
  motors.begin();
  ultrasonic.begin();
}

void loop() {
  // Check for obstacles before moving
  if (ultrasonic.isObstacleWithin(5)) {
    rgb.setRed();
    motors.stop();
    delay(1000);
    rgb.off();
    return;
  }
  
  rgb.setGreen();
  motors.forward(200);
  delay(500);
  
  motors.stop();
  rgb.off();
  delay(500);
  
  rgb.setRed();
  motors.reverse(200);
  delay(500);
  
  motors.stop();
  rgb.off();
  delay(500);
}
#include <Arduino.h>
#include "rgb_led.h"
#include "robot_drive.h"
#include "ultrasonic.h"
#include "command_receiver.h"
#include "command_handler.h"

RgbLed rgb;
MotorController motors;
UltrasonicSensor ultrasonic;
CommandReceiver receiver;

void setup() {
  receiver.begin(9600);
  rgb.begin();
  motors.begin();
  ultrasonic.begin();

  Serial.println("READY");
  rgb.setGreen();
  delay(500);
  rgb.off();
}

void loop() {
  if (receiver.hasCommand()) {
    String command = receiver.readCommand();
    processCommand(command);
  }
}
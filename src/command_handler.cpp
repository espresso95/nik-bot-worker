#include "command_handler.h"
#include "rgb_led.h"
#include "robot_drive.h"
#include "ultrasonic.h"

extern RgbLed rgb;
extern MotorController motors;
extern UltrasonicSensor ultrasonic;

typedef void (*CommandHandler)(const char* params);

struct Command {
  const char* name;
  CommandHandler handler;
};

void handleMove(const char* params) {
  int speed = atoi(params);
  if (ultrasonic.isObstacleWithin(5)) {
    rgb.setRed();
    motors.stop();
    Serial.println("ERR:OBSTACLE");
  } else {
    rgb.setGreen();
    motors.forward(speed);
    Serial.println("OK:MOVING");
  }
}

void handleReverse(const char* params) {
  int speed = atoi(params);
  rgb.setRed();
  motors.reverse(speed);
  Serial.println("OK:REVERSING");
}

void handleStop(const char* params) {
  motors.stop();
  rgb.off();
  Serial.println("OK:STOPPED");
}

void handleLeft(const char* params) {
  motors.turnLeft(150);
  Serial.println("OK:TURNING_LEFT");
}

void handleRight(const char* params) {
  motors.turnRight(150);
  Serial.println("OK:TURNING_RIGHT");
}

void handleLedRed(const char* params) {
  rgb.setRed();
  Serial.println("OK:LED_RED");
}

void handleLedGreen(const char* params) {
  rgb.setGreen();
  Serial.println("OK:LED_GREEN");
}

void handleLedBlue(const char* params) {
  rgb.setBlue();
  Serial.println("OK:LED_BLUE");
}

void handleLedOff(const char* params) {
  rgb.off();
  Serial.println("OK:LED_OFF");
}

void handleDistance(const char* params) {
  long distance = ultrasonic.readDistanceCm();
  Serial.print("DISTANCE:");
  Serial.println(distance);
}

void handleStatus(const char* params) {
  Serial.println("OK:READY");
}

const Command commands[] = {
  {"MOVE:", handleMove},
  {"REVERSE:", handleReverse},
  {"STOP", handleStop},
  {"LEFT", handleLeft},
  {"RIGHT", handleRight},
  {"LED:RED", handleLedRed},
  {"LED:GREEN", handleLedGreen},
  {"LED:BLUE", handleLedBlue},
  {"LED:OFF", handleLedOff},
  {"DISTANCE", handleDistance},
  {"STATUS", handleStatus}
};

const int numCommands = sizeof(commands) / sizeof(Command);

void processCommand(const String& cmd) {
  String trimmed = cmd;
  trimmed.trim();

  for (int i = 0; i < numCommands; i++) {
    if (trimmed.startsWith(commands[i].name)) {
      const char* params = trimmed.c_str() + strlen(commands[i].name);
      commands[i].handler(params);
      return;
    }
  }

  Serial.println("ERR:UNKNOWN_COMMAND");
}

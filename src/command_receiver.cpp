#include "command_receiver.h"

void CommandReceiver::begin(long baudRate) {
  Serial.begin(baudRate);
  inputBuffer.reserve(BUFFER_SIZE);
}

bool CommandReceiver::hasCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == DELIMITER) {
      return true;
    }
    if (c != '\r') {  // Ignore carriage return
      inputBuffer += c;
    }
  }
  return false;
}

String CommandReceiver::readCommand() {
  String command = inputBuffer;
  inputBuffer = "";
  return command;
}

void CommandReceiver::clearBuffer() {
  inputBuffer = "";
}

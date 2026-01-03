#ifndef COMMAND_RECEIVER_H
#define COMMAND_RECEIVER_H

#include <Arduino.h>

class CommandReceiver {
private:
  String inputBuffer;
  static const char DELIMITER = '\n';
  static const int BUFFER_SIZE = 64;

public:
  void begin(long baudRate = 9600);
  bool hasCommand();
  String readCommand();
  void clearBuffer();
};

#endif

#pragma once
#include <Arduino.h>
#include "pins.h"

// ESP32-CAM interface over UART
// The ESP32-CAM is a separate microcontroller that communicates with the Arduino Uno
// via UART (Serial) on pins D0 (RX) and D1 (TX).
// Note: Disconnect ESP32-CAM when uploading code to the Uno to avoid conflicts.
class Camera {
 public:
  Camera() = default;

  // Initialize UART communication with ESP32-CAM
  // baud_rate: Communication speed (default 115200)
  // init_delay_ms: Time to wait for ESP32-CAM to boot (default 1500ms)
  void begin(unsigned long baud_rate = 115200, uint16_t init_delay_ms = 1500) {
    Serial.begin(baud_rate);
    delay(init_delay_ms);  // Give ESP32-CAM time to initialize (typically 1-2 seconds)
  }

  // Send a command string to the ESP32-CAM
  void sendCommand(const String& command) {
    Serial.println(command);
  }

  // Check if data is available from ESP32-CAM
  bool isDataAvailable() {
    return Serial.available() > 0;
  }

  // Read a line of response from ESP32-CAM
  // timeout_ms: Maximum time to wait for a complete line (default 1000ms)
  // Returns empty string if no complete line is received within timeout
  String readResponse(uint16_t timeout_ms = 1000) {
    unsigned long start = millis();
    String response = "";
    
    while (millis() - start < timeout_ms) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
          return response;
        }
        response += c;
      }
    }
    return response;  // Return partial data or empty string on timeout
  }

  // Read raw bytes from ESP32-CAM
  // Returns number of bytes read
  size_t readBytes(uint8_t* buffer, size_t length) {
    return Serial.readBytes(buffer, length);
  }

  // Flush the serial buffer
  void flush() {
    Serial.flush();
  }

  // Common camera commands (customize based on ESP32-CAM firmware)

  // Request to capture a photo
  void capturePhoto() {
    sendCommand("CAPTURE");
  }

  // Request to start video streaming
  void startStream() {
    sendCommand("STREAM_START");
  }

  // Request to stop video streaming
  void stopStream() {
    sendCommand("STREAM_STOP");
  }

  // Request camera status
  void requestStatus() {
    sendCommand("STATUS");
  }

  // Set camera resolution
  // resolution: Resolution code (e.g., "QVGA", "VGA", "SVGA", etc.)
  void setResolution(const String& resolution) {
    sendCommand("RES_" + resolution);
  }

  // Set camera quality (JPEG quality)
  // quality: 0-63 (lower is higher quality, values >63 will be clamped)
  void setQuality(uint8_t quality) {
    if (quality > 63) {
      quality = 63;  // Clamp to valid range
    }
    sendCommand("QUALITY_" + String(quality));
  }

  // Enable/disable flash LED
  void setFlash(bool enable) {
    sendCommand(enable ? "FLASH_ON" : "FLASH_OFF");
  }

 private:
  // UART pins are defined in pins.h:
  // - Pins::kUartRx (D0)
  // - Pins::kUartTx (D1)
  // Serial object is used directly (no custom pins needed for Uno's hardware UART)
};

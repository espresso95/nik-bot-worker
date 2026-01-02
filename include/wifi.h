#pragma once
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "pins.h"

// WiFi bridge to ESP32-CAM module via UART
// The ESP32-CAM is connected to hardware serial (D0/D1) on the Zeus Car Shield.
// IMPORTANT: Disconnect ESP32-CAM or disable this component during sketch upload
// to avoid conflicts with USB programming which also uses hardware serial.
class WifiBridge {
 public:
  // Constructor for Hardware Serial (default, as ESP32-CAM is wired to D0/D1)
  // Note: serial_ptr will be set to &Serial in begin() when using hardware serial
  WifiBridge(long baud_rate = 9600)
      : baud_rate_(baud_rate), use_hardware_serial_(true),
        software_serial_(nullptr), connected_(false) {}
  
  // Constructor for Software Serial (alternative, for custom wiring)
  WifiBridge(uint8_t rx_pin, uint8_t tx_pin, long baud_rate = 9600)
      : rx_pin_(rx_pin), tx_pin_(tx_pin), baud_rate_(baud_rate),
        use_hardware_serial_(false), software_serial_(nullptr), connected_(false) {}

  ~WifiBridge() {
    if (software_serial_ != nullptr) {
      software_serial_->end();
      delete software_serial_;
    }
  }

  void begin() {
    if (use_hardware_serial_) {
      // Using hardware serial (D0/D1) - ESP32-CAM default connection
      Serial.begin(baud_rate_);
      delay(100);  // Allow ESP32 to stabilize
      
      // Clear any pending data
      while (Serial.available()) {
        Serial.read();
      }
    } else {
      // Using software serial on custom pins
      software_serial_ = new SoftwareSerial(rx_pin_, tx_pin_);
      software_serial_->begin(baud_rate_);
      delay(100);  // Allow ESP32 to stabilize
      
      // Clear any pending data
      while (software_serial_->available()) {
        software_serial_->read();
      }
    }
  }

  // Helper to get the active serial stream
  Stream* getSerial() {
    return use_hardware_serial_ ? (Stream*)&Serial : (Stream*)software_serial_;
  }

  // Send a command to ESP32 and wait for response
  bool sendCommand(const char* command, uint16_t timeout_ms = 1000) {
    Stream* serial = getSerial();
    if (serial == nullptr) return false;
    
    // Clear buffer
    while (serial->available()) {
      serial->read();
    }
    
    // Send command
    serial->println(command);
    
    // Wait for response
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
      if (serial->available()) {
        return true;
      }
      delay(10);
    }
    return false;
  }

  // Connect to WiFi network
  bool connect(const char* ssid, const char* password, uint16_t timeout_ms = 10000) {
    Stream* serial = getSerial();
    if (serial == nullptr) return false;
    
    // Validate input lengths to prevent buffer overflow
    // Command format: "WIFI_CONNECT:ssid,password" (15 chars overhead + ssid + password)
    if (strlen(ssid) > 32 || strlen(password) > 63) {
      return false;  // Invalid WiFi credentials length
    }
    
    // Format: WIFI_CONNECT:ssid,password
    char cmd[128];  // 15 + 32 + 63 + null = 111 bytes minimum
    snprintf(cmd, sizeof(cmd), "WIFI_CONNECT:%s,%s", ssid, password);
    
    if (sendCommand(cmd, timeout_ms)) {
      connected_ = waitForResponse("OK", timeout_ms);
      return connected_;
    }
    return false;
  }

  // Disconnect from WiFi
  void disconnect() {
    Stream* serial = getSerial();
    if (serial == nullptr) return;
    sendCommand("WIFI_DISCONNECT");
    connected_ = false;
  }

  // Check if connected to WiFi
  bool isConnected() const {
    return connected_;
  }

  // Get IP address (returns via response parameter)
  bool getIpAddress(char* ip_buffer, size_t buffer_size, uint16_t timeout_ms = 2000) {
    Stream* serial = getSerial();
    if (serial == nullptr || !connected_) return false;
    
    sendCommand("GET_IP");
    return readResponse(ip_buffer, buffer_size, timeout_ms);
  }

  // Send HTTP GET request
  bool httpGet(const char* url, char* response_buffer, size_t buffer_size, uint16_t timeout_ms = 5000) {
    Stream* serial = getSerial();
    if (serial == nullptr || !connected_) return false;
    
    // Validate URL length to prevent buffer overflow
    if (strlen(url) > 200) {
      return false;  // URL too long
    }
    
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "HTTP_GET:%s", url);
    
    if (sendCommand(cmd, timeout_ms)) {
      return readResponse(response_buffer, buffer_size, timeout_ms);
    }
    return false;
  }

  // Send HTTP POST request
  bool httpPost(const char* url, const char* data, char* response_buffer, size_t buffer_size, uint16_t timeout_ms = 5000) {
    Stream* serial = getSerial();
    if (serial == nullptr || !connected_) return false;
    
    // Validate combined length to prevent buffer overflow
    // Command format: "HTTP_POST:url|data" (11 chars overhead + url + data)
    if (strlen(url) + strlen(data) + 12 > 384) {
      return false;  // Combined URL and data too long for buffer
    }
    
    char cmd[384];  // 11 + url + data + null terminator
    snprintf(cmd, sizeof(cmd), "HTTP_POST:%s|%s", url, data);
    
    if (sendCommand(cmd, timeout_ms)) {
      return readResponse(response_buffer, buffer_size, timeout_ms);
    }
    return false;
  }

  // Read available data from serial
  int available() const {
    if (use_hardware_serial_) {
      return Serial.available();
    }
    return (software_serial_ != nullptr) ? software_serial_->available() : 0;
  }

  // Read a single byte
  int read() {
    if (use_hardware_serial_) {
      return Serial.read();
    }
    return (software_serial_ != nullptr) ? software_serial_->read() : -1;
  }

  // Write data to ESP32
  size_t write(const char* data) {
    if (use_hardware_serial_) {
      return Serial.print(data);
    }
    return (software_serial_ != nullptr) ? software_serial_->print(data) : 0;
  }

  // Read a line from serial (up to newline or buffer size)
  bool readLine(char* buffer, size_t buffer_size, uint16_t timeout_ms = 1000) {
    Stream* serial = getSerial();
    if (serial == nullptr) return false;
    
    unsigned long start = millis();
    size_t index = 0;
    
    while (millis() - start < timeout_ms && index < buffer_size - 1) {
      if (serial->available()) {
        char c = serial->read();
        if (c == '\n') {
          buffer[index] = '\0';
          return true;
        }
        if (c != '\r') {  // Skip carriage return
          buffer[index++] = c;
        }
      }
      delay(1);
    }
    
    buffer[index] = '\0';
    return index > 0;
  }

 private:
  // Wait for specific response string
  bool waitForResponse(const char* expected, uint16_t timeout_ms) {
    char buffer[64];
    unsigned long start = millis();
    
    while (millis() - start < timeout_ms) {
      if (readLine(buffer, sizeof(buffer), timeout_ms)) {
        if (strstr(buffer, expected) != nullptr) {
          return true;
        }
      }
    }
    return false;
  }

  // Read response into buffer
  bool readResponse(char* buffer, size_t buffer_size, uint16_t timeout_ms) {
    return readLine(buffer, buffer_size, timeout_ms);
  }

  uint8_t rx_pin_;
  uint8_t tx_pin_;
  long baud_rate_;
  bool use_hardware_serial_;
  SoftwareSerial* software_serial_;
  bool connected_;
};

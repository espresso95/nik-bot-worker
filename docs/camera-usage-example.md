# Camera Component Usage Example

This document provides examples of how to use the Camera component to communicate with the ESP32-CAM module.

## Important Notes

- The ESP32-CAM is a **separate microcontroller** that must be programmed independently
- The Camera class provides a communication interface from the Arduino Uno to the ESP32-CAM via UART
- **Always disconnect the ESP32-CAM when uploading code to the Arduino Uno** to avoid serial port conflicts
- The UART communication uses pins D0 (RX) and D1 (TX) on the Arduino Uno

## Basic Usage

### 1. Include the Camera Header

```cpp
#include <Arduino.h>
#include "camera.h"
```

### 2. Create a Camera Instance

```cpp
Camera camera;
```

### 3. Initialize in setup()

```cpp
void setup() {
  // Initialize with default settings (115200 baud, 1500ms boot delay)
  camera.begin();
  
  // Or customize the initialization
  // camera.begin(115200, 2000);  // 115200 baud, 2 second boot delay
}
```

### 4. Send Commands in loop()

```cpp
void loop() {
  // Capture a photo
  camera.capturePhoto();
  
  // Read response from ESP32-CAM (with 1 second timeout)
  String response = camera.readResponse(1000);
  if (response.length() > 0) {
    // Process response...
  }
  
  delay(5000);  // Wait 5 seconds before next capture
}
```

## Complete Example

Here's a complete example that integrates the camera with other components:

```cpp
#include <Arduino.h>
#include "camera.h"
#include "rgb_led.h"
#include "ultrasonic.h"

Camera camera;
RgbLed rgb;
UltrasonicSensor ultrasonic;

void setup() {
  // Initialize camera (this also initializes Serial)
  camera.begin();
  
  // Initialize other components
  rgb.begin();
  ultrasonic.begin();
  
  // Set initial status LED
  rgb.setBlue();
  delay(1000);
  rgb.off();
}

void loop() {
  // Check for obstacles
  if (ultrasonic.isObstacleWithin(20)) {
    // Obstacle detected - capture photo and set LED to red
    rgb.setRed();
    camera.capturePhoto();
    
    // Wait for confirmation from ESP32-CAM
    String response = camera.readResponse(500);
    if (response.length() > 0) {
      // Process response...
    }
    
    delay(2000);
    rgb.off();
  }
  
  delay(100);
}
```

## Available Methods

### Initialization

- `begin(unsigned long baud_rate = 115200, uint16_t init_delay_ms = 1500)` - Initialize UART communication with configurable baud rate and boot delay

### Communication

- `sendCommand(const String& command)` - Send a custom command string
- `isDataAvailable()` - Check if data is available from ESP32-CAM
- `readResponse(uint16_t timeout_ms = 1000)` - Read a line of text response with timeout
- `readBytes(uint8_t* buffer, size_t length)` - Read raw bytes
- `flush()` - Flush the serial buffer

### Camera Control Commands

These commands send predefined strings to the ESP32-CAM. The actual behavior depends on the ESP32-CAM firmware:

- `capturePhoto()` - Request photo capture (sends "CAPTURE")
- `startStream()` - Start video streaming (sends "STREAM_START")
- `stopStream()` - Stop video streaming (sends "STREAM_STOP")
- `requestStatus()` - Request camera status (sends "STATUS")
- `setResolution(const String& resolution)` - Set resolution (sends "RES_<resolution>")
- `setQuality(uint8_t quality)` - Set JPEG quality 0-63, values >63 are clamped (sends "QUALITY_<value>")
- `setFlash(bool enable)` - Enable/disable flash LED (sends "FLASH_ON" or "FLASH_OFF")

## Custom Commands

You can send custom commands to match your ESP32-CAM firmware:

```cpp
// Send a custom command
camera.sendCommand("MY_CUSTOM_COMMAND");

// Send a command with parameters
camera.sendCommand("SET_BRIGHTNESS 128");
```

## ESP32-CAM Firmware Requirements

The Camera component sends text commands over UART. Your ESP32-CAM firmware should:

1. Listen on the UART for incoming commands
2. Parse the command strings
3. Execute the appropriate camera operations
4. Optionally send responses back to the Arduino Uno

Example ESP32-CAM firmware structure:

```cpp
// On ESP32-CAM side
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    
    if (command == "CAPTURE") {
      // Capture photo and save/send
      captureAndSavePhoto();
      Serial.println("OK:CAPTURED");
    }
    else if (command == "STATUS") {
      Serial.println("OK:READY");
    }
    // Add more command handlers...
  }
}
```

## Troubleshooting

### Upload Fails

If you get upload errors when trying to program the Arduino Uno:
- **Disconnect the ESP32-CAM** from the shield before uploading
- The ESP32-CAM shares the same UART pins used for programming

### No Response from Camera

- Verify the ESP32-CAM is powered and programmed with compatible firmware
- Check the baud rate matches between Arduino and ESP32-CAM
- Ensure proper wiring between Arduino Uno and ESP32-CAM

### Serial Monitor Issues

- When using the Camera component, the hardware Serial port is used for ESP32-CAM communication
- If you need debugging output, consider using SoftwareSerial on different pins
- Alternatively, temporarily disable camera communication to use Serial for debugging

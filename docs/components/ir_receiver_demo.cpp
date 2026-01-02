// IR receiver demo (HS0038B on D2). Captures NEC frames and prints decoded 32-bit code.
#include <Arduino.h>
#include "../../include/pins.h"

struct IrNec {
  static constexpr uint8_t MAX_PULSES = 100;
  volatile uint16_t widths[MAX_PULSES];
  volatile uint8_t count = 0;
  volatile unsigned long lastEdgeUs = 0;
  volatile bool frameReady = false;

  void begin() {
    pinMode(Pins::kIrReceiver, INPUT);
    count = 0; frameReady = false; lastEdgeUs = micros();
    attachInterrupt(digitalPinToInterrupt(Pins::kIrReceiver), isrThunk, CHANGE);
    instance = this;
  }

  bool poll(uint32_t &codeOut) {
    if (!frameReady) return false;
    noInterrupts();
    uint8_t n = count; uint16_t local[MAX_PULSES];
    for (uint8_t i = 0; i < n; i++) local[i] = widths[i];
    count = 0; frameReady = false; interrupts();

    if (n < 66) return false; // leader + 32 bits (edges)
    auto near = [](uint16_t v, uint16_t t, uint16_t tol) { return v > t - tol && v < t + tol; };

    int start = -1;
    for (int i = 0; i < (int)n - 2; i++) if (near(local[i], 9000, 2000) && near(local[i+1], 4500, 1500)) { start = i + 2; break; }
    if (start < 0) return false;

    uint32_t code = 0; int bit = 0;
    for (int i = start; i + 1 < (int)n && bit < 32; i += 2) {
      uint16_t mark = local[i];
      uint16_t space = local[i+1];
      if (!near(mark, 560, 300)) continue;
      code <<= 1;
      if (near(space, 1690, 600)) code |= 1; else if (!near(space, 560, 300)) return false;
      bit++;
    }
    if (bit != 32) return false;
    codeOut = code;
    return true;
  }

private:
  static IrNec *instance;
  static void isrThunk() { if (instance) instance->isr(); }
  void isr() {
    unsigned long now = micros();
    uint16_t w = (uint16_t)min<unsigned long>(65535, now - lastEdgeUs);
    lastEdgeUs = now;
    if (w > 15000 && count > 10) { frameReady = true; return; }
    if (count < MAX_PULSES) widths[count++] = w; else count = 0;
  }
};
IrNec* IrNec::instance = nullptr;

IrNec ir;

void setup() {
  Serial.begin(115200);
  ir.begin();
  Serial.println("IR demo: point NEC remote at receiver; printing codes");
}

void loop() {
  uint32_t code;
  if (ir.poll(code)) {
    Serial.print("NEC code: 0x");
    Serial.println(code, HEX);
  }
}

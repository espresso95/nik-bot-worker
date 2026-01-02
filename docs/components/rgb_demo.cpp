// RGB strip demo (Zeus kit pins on 12, 13, 11; uses software PWM for non-PWM pins)
#include <Arduino.h>
#include "../../include/pins.h"

namespace softpwm {
  static constexpr uint8_t MAX_CH = 8;
  static volatile uint8_t phase = 0;
  struct Channel { uint8_t pin = 255; volatile uint8_t duty = 0; bool active = false; };
  static Channel ch[MAX_CH];
  static int findPin(uint8_t pin) { for (uint8_t i = 0; i < MAX_CH; i++) if (ch[i].active && ch[i].pin == pin) return i; return -1; }
  static int alloc(uint8_t pin) { int idx = findPin(pin); if (idx >= 0) return idx; for (uint8_t i = 0; i < MAX_CH; i++) if (!ch[i].active) { ch[i].active = true; ch[i].pin = pin; ch[i].duty = 0; pinMode(pin, OUTPUT); digitalWrite(pin, LOW); return i; } return -1; }
  void write(uint8_t pin, uint8_t duty) { int idx = alloc(pin); if (idx >= 0) ch[idx].duty = duty; }
  void begin() { cli(); TCCR2A = 0; TCCR2B = 0; TCCR2B |= (1 << CS22); TIMSK2 |= (1 << TOIE2); sei(); }
  ISR(TIMER2_OVF_vect) { phase++; for (uint8_t i = 0; i < MAX_CH; i++) if (ch[i].active) digitalWrite(ch[i].pin, (phase < ch[i].duty) ? HIGH : LOW); }
}

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  softpwm::write(Pins::kRgbRed, r);
  softpwm::write(Pins::kRgbGreen, g);
  softpwm::write(Pins::kRgbBlue, b);
}

void setup() {
  Serial.begin(115200);
  softpwm::begin();
  setRgb(0, 0, 0);
  Serial.println("RGB demo: cycling R/G/B and white");
}

void loop() {
  setRgb(255, 0, 0); delay(800);
  setRgb(0, 255, 0); delay(800);
  setRgb(0, 0, 255); delay(800);
  setRgb(255, 255, 255); delay(800);
}

// Motors demo using Zeus Car Shield pins. Drives forward 2s, reverse 2s, then stops.
#include <Arduino.h>
#include "../../include/pins.h"
#include "demos.h"

namespace motors_demo {
namespace softpwm {
  static constexpr uint8_t MAX_CH = 16;
  static volatile uint8_t phase = 0;
  struct Channel { uint8_t pin = 255; volatile uint8_t duty = 0; bool active = false; };
  static Channel ch[MAX_CH];

  static int findPin(uint8_t pin) {
    for (uint8_t i = 0; i < MAX_CH; i++) if (ch[i].active && ch[i].pin == pin) return i;
    return -1;
  }
  static int alloc(uint8_t pin) {
    int idx = findPin(pin);
    if (idx >= 0) return idx;
    for (uint8_t i = 0; i < MAX_CH; i++) if (!ch[i].active) { ch[i].active = true; ch[i].pin = pin; ch[i].duty = 0; pinMode(pin, OUTPUT); digitalWrite(pin, LOW); return i; }
    return -1;
  }
  void write(uint8_t pin, uint8_t duty) { int idx = alloc(pin); if (idx >= 0) ch[idx].duty = duty; }
  void begin() { cli(); TCCR2A = 0; TCCR2B = 0; TCCR2B |= (1 << CS22); TIMSK2 |= (1 << TOIE2); sei(); }
  ISR(TIMER2_OVF_vect) { phase++; for (uint8_t i = 0; i < MAX_CH; i++) if (ch[i].active) digitalWrite(ch[i].pin, (phase < ch[i].duty) ? HIGH : LOW); }
}

struct Motor2Pin {
  uint8_t in1 = 0, in2 = 0;
  void begin(uint8_t p1, uint8_t p2) { in1 = p1; in2 = p2; softpwm::write(in1, 0); softpwm::write(in2, 0); }
  void set(int16_t speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) { softpwm::write(in1, (uint8_t)speed); softpwm::write(in2, 0); }
    else if (speed < 0) { softpwm::write(in1, 0); softpwm::write(in2, (uint8_t)(-speed)); }
    else { softpwm::write(in1, 0); softpwm::write(in2, 0); }
  }
};

struct DriveTrain {
  Motor2Pin fl, fr, rl, rr;
  void begin() {
    fl.begin(Pins::kMotorOutA1, Pins::kMotorOutB1);
    fr.begin(Pins::kMotorOutA2, Pins::kMotorOutB2);
    rl.begin(Pins::kMotorOutA3, Pins::kMotorOutB3);
    rr.begin(Pins::kMotorOutA4, Pins::kMotorOutB4);
  }
  void setWheels(int16_t sFL, int16_t sFR, int16_t sRL, int16_t sRR) { fl.set(sFL); fr.set(sFR); rl.set(sRL); rr.set(sRR); }
};

DriveTrain drive;
unsigned long t0 = 0;
}

void motorsDemoSetup() {
  using namespace motors_demo;
  Serial.begin(115200);
  softpwm::begin();
  drive.begin();
  t0 = millis();
  Serial.println("Motors demo: fwd 2s, rev 2s, then stop");
}

void motorsDemoLoop() {
  using namespace motors_demo;
  unsigned long now = millis();
  if (now - t0 < 2000) {
    drive.setWheels(180, 180, 180, 180);
  } else if (now - t0 < 4000) {
    drive.setWheels(-180, -180, -180, -180);
  } else {
    drive.setWheels(0, 0, 0, 0);
  }
}

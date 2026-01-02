Below is a **single PlatformIO “bring-up” project** that touches (and proves) *all the Uno-side components* in the Zeus kit:

* **4 motors** via the Zeus Car Shield motor pins ([SunFounder Documentation][1])
* **Ultrasonic** (Trig+Echo on **D10**, one-pin mode) ([SunFounder Documentation][1])
* **Omni grayscale + 2 IR obstacle sensors** through the **74HC165** chain (D7/D8/D9) ([SunFounder Documentation][1])
* **RGB strips** on pins **12, 13, 11** ([SunFounder Documentation][2])
* **IR receiver** on **D2** ([SunFounder Documentation][1])

> Note on the **ESP32-CAM**: it’s a separate microcontroller. This sample doesn’t “run the camera” from the Uno (you’d program ESP32-CAM separately). Also, the kit wiring shares Uno serial lines with ESP32-CAM, so **disconnect the ESP32-CAM while uploading** if you get upload errors. ([SunFounder Documentation][3])

---

## 1) `platformio.ini`

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 115200
```

---

## 2) `include/pins_zeus.h`

```cpp
#pragma once
#include <Arduino.h>

// Pin mappings per SunFounder Zeus Car Shield docs
// Motors: ~3,4,~5,~6,A0,A1,A2,A3  :contentReference[oaicite:6]{index=6}
// Shift register: D7 (DATA), D8 (CLK), D9 (LATCH) :contentReference[oaicite:7]{index=7}
// Ultrasonic: D10 (Trig+Echo) :contentReference[oaicite:8]{index=8}
// IR receiver: D2 :contentReference[oaicite:9]{index=9}
// RGB strip: pins 12,13,11 :contentReference[oaicite:10]{index=10}

namespace pins {
  // Motor control pins (shield labels)
  constexpr uint8_t OUTA1 = 3;   // ~D3
  constexpr uint8_t OUTB1 = 4;   // D4
  constexpr uint8_t OUTA2 = 5;   // ~D5
  constexpr uint8_t OUTB2 = 6;   // ~D6
  constexpr uint8_t OUTB4 = A0;  // A0
  constexpr uint8_t OUTA4 = A1;  // A1
  constexpr uint8_t OUTB3 = A2;  // A2
  constexpr uint8_t OUTA3 = A3;  // A3

  // 74HC165 chain (grayscale + obstacle)
  constexpr uint8_t SR_DATA  = 7;  // Q7
  constexpr uint8_t SR_CLOCK = 8;  // CP
  constexpr uint8_t SR_LATCH = 9;  // PL

  // Ultrasonic (one pin)
  constexpr uint8_t ULTRASONIC_1PIN = 10;

  // IR receiver
  constexpr uint8_t IR_RX = 2;

  // RGB strip channels (common anode strips, driven by shield)
  constexpr uint8_t RGB_G = 12;
  constexpr uint8_t RGB_R = 13;
  constexpr uint8_t RGB_B = 11;
}
```

---

## 3) `src/main.cpp` (all components demo)

```cpp
#include <Arduino.h>
#include "pins_zeus.h"

/*
  Zeus bring-up demo:
  - Software PWM for ALL output pins (so even non-hardware-PWM pins can do speed/brightness)
  - 4 motor control (mecanum mixing helpers)
  - 74HC165 read (grayscale + 2 obstacle bits)
  - one-pin ultrasonic on D10
  - RGB channels
  - very small NEC-ish IR receiver decoder (basic; good enough for key presses)

  Serial commands (115200):
    HELP
    LED r g b              (0-255 each)
    DRIVE vx vy w          (-255..255 each)  vx=fwd, vy=strafe, w=rotate
    WHEELS fl fr rl rr     (-255..255 each) direct wheel command
    STOP
    SENS                   prints grayscale+obstacle bits
    PING                   prints ultrasonic distance
*/

//
// ---------- SoftPWM (Timer2) ----------
//
namespace softpwm {
  static constexpr uint8_t MAX_CH = 16;
  static volatile uint8_t phase = 0;

  struct Channel {
    uint8_t pin = 255;
    volatile uint8_t duty = 0; // 0..255
    bool active = false;
  };

  static Channel ch[MAX_CH];

  static int findPin(uint8_t pin) {
    for (uint8_t i = 0; i < MAX_CH; i++) {
      if (ch[i].active && ch[i].pin == pin) return i;
    }
    return -1;
  }

  static int alloc(uint8_t pin) {
    int idx = findPin(pin);
    if (idx >= 0) return idx;
    for (uint8_t i = 0; i < MAX_CH; i++) {
      if (!ch[i].active) {
        ch[i].active = true;
        ch[i].pin = pin;
        ch[i].duty = 0;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        return i;
      }
    }
    return -1;
  }

  void write(uint8_t pin, uint8_t duty) {
    int idx = alloc(pin);
    if (idx < 0) return;
    ch[idx].duty = duty;
  }

  void begin() {
    // Timer2 overflow ~976 Hz: 16MHz / 64 / 256
    cli();
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B |= (1 << CS22); // prescaler 64
    TIMSK2 |= (1 << TOIE2);
    sei();
  }

  ISR(TIMER2_OVF_vect) {
    phase++; // 0..255
    for (uint8_t i = 0; i < MAX_CH; i++) {
      if (!ch[i].active) continue;
      // Basic PWM: high when phase < duty
      digitalWrite(ch[i].pin, (phase < ch[i].duty) ? HIGH : LOW);
    }
  }
} // namespace softpwm

//
// ---------- RGB (3-channel) ----------
//
struct Rgb {
  // If your colors look inverted, flip these in set()
  void begin() {
    softpwm::write(pins::RGB_R, 0);
    softpwm::write(pins::RGB_G, 0);
    softpwm::write(pins::RGB_B, 0);
  }
  void set(uint8_t r, uint8_t g, uint8_t b) {
    softpwm::write(pins::RGB_R, r);
    softpwm::write(pins::RGB_G, g);
    softpwm::write(pins::RGB_B, b);
  }
};

//
// ---------- Motors (2-pin per motor, signed speed) ----------
//
struct Motor2Pin {
  uint8_t in1, in2; // drive pins
  void begin(uint8_t p1, uint8_t p2) { in1 = p1; in2 = p2; softpwm::write(in1, 0); softpwm::write(in2, 0); }

  // speed: -255..255
  void set(int16_t speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) { softpwm::write(in1, (uint8_t)speed); softpwm::write(in2, 0); }
    else if (speed < 0) { softpwm::write(in1, 0); softpwm::write(in2, (uint8_t)(-speed)); }
    else { softpwm::write(in1, 0); softpwm::write(in2, 0); } // coast
  }

  void brake() { softpwm::write(in1, 255); softpwm::write(in2, 255); }
};

struct DriveTrain {
  // Wheel naming: FL, FR, RL, RR (you may need to swap/flip signs depending on how you installed motors)
  Motor2Pin fl, fr, rl, rr;

  void begin() {
    // Map motor ports to wheel positions.
    // These are *reasonable defaults* based on shield pin labels; adjust if your wheels don't match.
    // Motor Port 1: OUTA1/OUTB1, Port 2: OUTA2/OUTB2, Port 3: OUTA3/OUTB3, Port 4: OUTA4/OUTB4 :contentReference[oaicite:11]{index=11}
    fl.begin(pins::OUTA1, pins::OUTB1);
    fr.begin(pins::OUTA2, pins::OUTB2);
    rl.begin(pins::OUTA3, pins::OUTB3);
    rr.begin(pins::OUTA4, pins::OUTB4);
  }

  void stop(bool brake=false) {
    if (brake) { fl.brake(); fr.brake(); rl.brake(); rr.brake(); }
    else { fl.set(0); fr.set(0); rl.set(0); rr.set(0); }
  }

  void setWheels(int16_t sFL, int16_t sFR, int16_t sRL, int16_t sRR) {
    fl.set(sFL); fr.set(sFR); rl.set(sRL); rr.set(sRR);
  }

  // Holonomic mix (mecanum). vx=fwd, vy=strafe right, w=rotate clockwise
  void drive(int16_t vx, int16_t vy, int16_t w) {
    long flv = (long)vx - (long)vy - (long)w;
    long frv = (long)vx + (long)vy + (long)w;
    long rlv = (long)vx + (long)vy - (long)w;
    long rrv = (long)vx - (long)vy + (long)w;

    long m = max(max(labs(flv), labs(frv)), max(labs(rlv), labs(rrv)));
    if (m > 255) {
      flv = flv * 255 / m;
      frv = frv * 255 / m;
      rlv = rlv * 255 / m;
      rrv = rrv * 255 / m;
    }
    setWheels((int16_t)flv, (int16_t)frv, (int16_t)rlv, (int16_t)rrv);
  }
};

//
// ---------- 74HC165 chain: grayscale (8 bits) + obstacle (2 bits) ----------
// According to SunFounder docs: first 8 bits are grayscale, last two bits are obstacle bits. :contentReference[oaicite:12]{index=12}
struct Hc165 {
  void begin() {
    pinMode(pins::SR_DATA, INPUT);
    pinMode(pins::SR_CLOCK, OUTPUT);
    pinMode(pins::SR_LATCH, OUTPUT);
    digitalWrite(pins::SR_CLOCK, LOW);
    digitalWrite(pins::SR_LATCH, HIGH);
  }

  // Read N bits MSB-first as they come off Q7
  uint16_t readBits(uint8_t nbits) {
    // latch parallel inputs
    digitalWrite(pins::SR_LATCH, LOW);
    delayMicroseconds(5);
    digitalWrite(pins::SR_LATCH, HIGH);
    delayMicroseconds(5);

    uint16_t v = 0;
    for (uint8_t i = 0; i < nbits; i++) {
      v <<= 1;
      v |= digitalRead(pins::SR_DATA) ? 1 : 0;

      digitalWrite(pins::SR_CLOCK, HIGH);
      delayMicroseconds(2);
      digitalWrite(pins::SR_CLOCK, LOW);
      delayMicroseconds(2);
    }
    return v;
  }

  struct Sample {
    uint8_t grayscale;  // 8 bits, 1=white, 0=black (typical)
    uint8_t obstacle2;  // two bits (S1,S0)
  };

  Sample readSample() {
    // read 10 meaningful bits: 8 grayscale + 2 obstacle
    uint16_t v10 = readBits(10);
    Sample s{};
    // "first 8 bits are grayscale, last 2 are obstacle" => top 8 bits of v10 = grayscale
    s.grayscale = (uint8_t)(v10 >> 2);
    s.obstacle2 = (uint8_t)(v10 & 0x03);
    return s;
  }
};

//
// ---------- Ultrasonic one-pin (D10 trig+echo) ----------
//
struct Ultrasonic1Pin {
  uint8_t pin;
  void begin(uint8_t p) {
    pin = p;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Returns cm, or -1 on timeout
  long readCm(uint16_t timeout_us = 25000) { // ~4m max
    // trigger pulse
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);

    // read echo on same pin
    pinMode(pin, INPUT);

    // wait for HIGH
    unsigned long start = micros();
    while (digitalRead(pin) == LOW) {
      if ((micros() - start) > timeout_us) return -1;
    }

    // measure HIGH width
    unsigned long echoStart = micros();
    while (digitalRead(pin) == HIGH) {
      if ((micros() - echoStart) > timeout_us) return -1;
    }
    unsigned long echoUs = micros() - echoStart;

    // Sound speed ~343m/s => 29.1 us per cm round-trip => cm = echoUs / 58
    return (long)(echoUs / 58UL);
  }
};

//
// ---------- IR receiver: minimal NEC-ish decoder on D2 ----------
// This is a basic decoder for learning / key presses. For “production”, use an IR library.
// Many IR receivers idle HIGH and output active-low bursts.
struct IrNec {
  // ISR-captured pulse widths
  static constexpr uint8_t MAX_PULSES = 100;
  volatile uint16_t widths[MAX_PULSES];
  volatile uint8_t count = 0;
  volatile unsigned long lastEdgeUs = 0;
  volatile bool frameReady = false;

  uint32_t lastCode = 0;
  bool hasCode = false;

  void begin() {
    pinMode(pins::IR_RX, INPUT);
    count = 0;
    frameReady = false;
    lastEdgeUs = micros();
    attachInterrupt(digitalPinToInterrupt(pins::IR_RX), isrThunk, CHANGE);
    instance = this;
  }

  // Call in loop; returns true if a new code is available
  bool poll(uint32_t &codeOut) {
    if (!frameReady) return false;

    noInterrupts();
    uint8_t n = count;
    uint16_t local[MAX_PULSES];
    for (uint8_t i = 0; i < n; i++) local[i] = widths[i];
    count = 0;
    frameReady = false;
    interrupts();

    // crude NEC parse:
    // expect leader ~9000us + 4500us, then 32 bits (each bit ~560+560 or ~560+1690)
    // Because we captured CHANGE edges, widths alternate high/low durations; exact polarity depends.
    if (n < 2 + 64) return false;

    auto near = [](uint16_t v, uint16_t target, uint16_t tol) { return (v > target - tol) && (v < target + tol); };

    // Try to find leader pair anywhere in the capture
    int startIdx = -1;
    for (int i = 0; i < (int)n - 2; i++) {
      if (near(local[i], 9000, 2000) && near(local[i+1], 4500, 1500)) { startIdx = i+2; break; }
    }
    if (startIdx < 0) return false;

    uint32_t code = 0;
    int bit = 0;

    // NEC bits: mark ~560, space ~560 (0) or ~1690 (1)
    for (int i = startIdx; i+1 < (int)n && bit < 32; i += 2) {
      uint16_t mark = local[i];
      uint16_t space = local[i+1];
      if (!near(mark, 560, 300)) continue; // skip noise
      code <<= 1;
      if (near(space, 1690, 600)) code |= 1;
      else if (near(space, 560, 300)) code |= 0;
      else return false;
      bit++;
    }
    if (bit != 32) return false;

    lastCode = code;
    hasCode = true;
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

    // If we have a long gap, treat as end-of-frame
    if (w > 15000 && count > 10) {
      frameReady = true;
      return;
    }

    if (count < MAX_PULSES) {
      widths[count++] = w;
    } else {
      // overflow; reset
      count = 0;
    }
  }
};
IrNec* IrNec::instance = nullptr;

//
// ---------- Global instances ----------
//
Rgb rgb;
DriveTrain drive;
Hc165 hc165;
Ultrasonic1Pin sonar;
IrNec ir;

//
// ---------- Simple serial command parsing ----------
//
static String line;

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  HELP"));
  Serial.println(F("  LED r g b              (0-255)"));
  Serial.println(F("  DRIVE vx vy w          (-255..255) vx=fwd, vy=strafe, w=rotate"));
  Serial.println(F("  WHEELS fl fr rl rr     (-255..255)"));
  Serial.println(F("  STOP"));
  Serial.println(F("  SENS"));
  Serial.println(F("  PING"));
}

static bool readInt3(const char *s, int &a, int &b, int &c) {
  return (sscanf(s, "%d %d %d", &a, &b, &c) == 3);
}

static bool readInt4(const char *s, int &a, int &b, int &c, int &d) {
  return (sscanf(s, "%d %d %d %d", &a, &b, &c, &d) == 4);
}

static void handleLine(const String &cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "HELP") { printHelp(); return; }
  if (cmd == "STOP") { drive.stop(false); rgb.set(0, 0, 0); return; }

  if (cmd.startsWith("LED ")) {
    int r,g,b;
    if (readInt3(cmd.c_str()+4, r,g,b)) rgb.set((uint8_t)constrain(r,0,255), (uint8_t)constrain(g,0,255), (uint8_t)constrain(b,0,255));
    else Serial.println(F("ERR LED"));
    return;
  }

  if (cmd.startsWith("DRIVE ")) {
    int vx,vy,w;
    if (readInt3(cmd.c_str()+6, vx,vy,w)) drive.drive((int16_t)vx, (int16_t)vy, (int16_t)w);
    else Serial.println(F("ERR DRIVE"));
    return;
  }

  if (cmd.startsWith("WHEELS ")) {
    int fl,fr,rl,rr;
    if (readInt4(cmd.c_str()+7, fl,fr,rl,rr)) drive.setWheels((int16_t)fl,(int16_t)fr,(int16_t)rl,(int16_t)rr);
    else Serial.println(F("ERR WHEELS"));
    return;
  }

  if (cmd == "SENS") {
    auto s = hc165.readSample();
    Serial.print(F("grayscale="));
    for (int i = 7; i >= 0; i--) Serial.print((s.grayscale >> i) & 1);
    Serial.print(F(" obstacle2="));
    Serial.println(s.obstacle2, BIN);
    return;
  }

  if (cmd == "PING") {
    long cm = sonar.readCm();
    Serial.print(F("cm="));
    Serial.println(cm);
    return;
  }

  Serial.println(F("Unknown. Type HELP"));
}

//
// ---------- Setup / loop ----------
//
void setup() {
  Serial.begin(115200);

  softpwm::begin();     // PWM engine first
  rgb.begin();
  drive.begin();
  hc165.begin();
  sonar.begin(pins::ULTRASONIC_1PIN);
  ir.begin();

  Serial.println(F("\nZeus bring-up (Uno)"));
  printHelp();
  rgb.set(0, 20, 0); // dim green = alive
}

void loop() {
  // Read serial lines
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleLine(line);
      line = "";
    } else if (line.length() < 80) {
      line += c;
    }
  }

  // IR remote events
  uint32_t code;
  if (ir.poll(code)) {
    Serial.print(F("IR NEC code: 0x"));
    Serial.println(code, HEX);

    // Tiny demo mapping: any code toggles a fun LED color / stop
    rgb.set((uint8_t)(code & 0xFF), (uint8_t)((code >> 8) & 0xFF), (uint8_t)((code >> 16) & 0xFF));
  }

  // Periodic telemetry (every ~500ms)
  static unsigned long t0 = 0;
  if (millis() - t0 > 500) {
    t0 = millis();
    auto s = hc165.readSample();
    long cm = sonar.readCm(15000);

    Serial.print(F("T sens grayscale="));
    Serial.print(s.grayscale, BIN);
    Serial.print(F(" obs="));
    Serial.print(s.obstacle2, BIN);
    Serial.print(F(" cm="));
    Serial.println(cm);
  }
}
```

---

## What you’ll do with this (quick checklist)

1. **Upload** in PlatformIO. If upload fails, **unplug/disconnect the ESP32-CAM** and try again. ([SunFounder Documentation][3])
2. Open Serial Monitor at **115200**.
3. Try:

   * `LED 255 0 0` (red), `LED 0 255 0` (green), `LED 0 0 255` (blue)
   * `PING` (distance)
   * `SENS` (prints grayscale bits + obstacle bits)
   * `DRIVE 120 0 0` (forward), `DRIVE 0 120 0` (strafe), `DRIVE 0 0 120` (rotate)
4. If wheels are “wrong” (e.g., forward strafes), adjust the wheel mapping inside `DriveTrain::begin()` or flip signs in `drive()`.
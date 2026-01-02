# nik-bot

## Build, upload, and monitor (PlatformIO)

1) Build: `pio run`

2) Upload to Arduino Uno R3 (USB): `pio run -t upload`

3) Open serial monitor at 9600 baud: `pio device monitor -b 9600`

If the upload succeeds, the onboard LED should blink and the monitor will print heartbeat messages.

## Components (Zeus kit bring-up)
| Component | Notes / Pins |
| --- | --- |
| Arduino Uno R3 (SunFounder R3) | Main MCU flashed via PlatformIO |
| Zeus Car Shield + 4 DC motors | Mecanum drive; motor ports on shield (OUTA/OUTB) |
| Ultrasonic sensor | One-pin Trig/Echo on D10 |
| Omni grayscale array + 2 IR obstacle sensors | Read through 74HC165 on D7 (data), D8 (clock), D9 (latch) |
| RGB LED strips | Channels on pins 12 (G), 13 (R), 11 (B) |
| IR receiver | Connected to D2 |
| ESP32-CAM module | Program separately; unplug from Uno serial when uploading if needed |
# PID Line Following Robot — ESP32

> Autonomous line following robot with 8-channel IR sensing, real-time discrete PID control, and live WiFi-based gain tuning — running on ESP32's dual-core architecture.

This is one of the projects from my personal robotics portfolio. Built entirely from scratch — firmware, PID tuning, calibration logic, and the live WiFi dashboard are all hand-written and tested on real hardware.

---

## What makes this different from a basic line follower

Most line followers run everything on a single core, which means any serial logging or WiFi handling steals time from the control loop. On ESP32 the PID loop runs exclusively on Core 1 while the WiFi tuning dashboard runs independently on Core 0. The control loop never waits on anything else.

The other big thing is live tuning. Instead of changing a value in code, reflashing, and running again — you connect your phone to the robot's WiFi hotspot and adjust Kp, Kd, Ki, and speed from a browser while the robot is actually moving on the track. This cuts tuning time dramatically.

---

## Features

- Full PID control loop — proportional, integral, derivative
- 8-channel IR sensor array with per-sensor adaptive calibration
- Weighted sensor averaging for smooth continuous error signal
- **Dual-core execution** — PID on Core 1, WiFi on Core 0
- **Live browser tuning dashboard** — adjust gains without reflashing
- Anti-windup clamp on integrator (±500)
- Integrator resets to zero on line loss
- Bidirectional calibration sweep for any lighting condition
- Smooth speed ramp from standstill to cruise
- Coast-to-stop motor behaviour
- Line-loss recovery using last known error direction
- Black-on-white or white-on-black configurable

---

## Hardware

| Component | Part |
|---|---|
| Microcontroller | ESP32 DevKit V1 (dual-core, 240 MHz) |
| Motor Driver | TB6612FNG Dual H-Bridge |
| Sensor Array | 8x IR analog sensors |
| Drive Motors | 2x DC gear motors |
| Button 1 (GPIO 18) | Trigger calibration |
| Button 2 (GPIO 19) | Start run |
| LED (GPIO 2) | On-line indicator (onboard) |

---

## Wiring

---

## Live WiFi Tuning

On boot the ESP32 creates a WiFi access point:
Connect from your phone or laptop, open the URL, and you get a live dashboard with sliders for Kp, Kd, Ki, and top speed. Hit Apply and the values update instantly on the running robot — no stopping, no reflashing.

---

## Dual-Core Architecture
Gains are declared volatile so both cores read and write them safely without race conditions.

---

## PID Control System
### Sensor Weighting
### Tuned Parameters

| Gain | Value | Effect |
|---|---|---|
| Kp | 0.13 | Steering strength per unit of error |
| Kd | 0.40 | Damping — kills oscillation at high speed |
| Ki | 0.0005 | Eliminates residual drift on long straights |
| lfSpeed | 220 | Cruise speed (0-255 PWM) |
| Start Speed | 60 | Ramps up each iteration — no wheel slip |

---

## ESP32-Specific Implementation Notes

**PWM** — ESP32 has no analogWrite(). Motor speed is controlled via the LEDC peripheral using ledcWrite(), configured at 1 kHz / 8-bit resolution.

**ADC** — ESP32 ADC is 12-bit (0-4095). Sensor values are mapped to 0-1000 to keep them consistent with the PID weight scale.

**ADC2 / WiFi conflict** — ESP32's ADC2 pins are disabled when WiFi is active. All 8 sensor pins are assigned to ADC1 in the firmware to avoid this.

**Dual core** — The WiFi server task is pinned to Core 0 using xTaskCreatePinnedToCore(). Core 1 runs the control loop with no interference.

---

## How to Run

1. Install ESP32 board package in Arduino IDE
   - File -> Preferences -> add https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   - Tools -> Board -> ESP32 Dev Module
2. Flash PID_LineFollower_ESP32.ino
3. Open Serial Monitor at 115200 baud
4. Place robot on line -> press Button 1 -> calibration sweep (~10 sec)
5. Press Button 2 -> robot starts following
6. Connect phone to PID-LineFollower WiFi -> open http://192.168.4.1 -> tune live

---

## Tuning Guide

**Oscillating at speed** — raise Kd first, try 0.45 then 0.50

**Slow on sharp corners** — raise Kp to 0.15

**Drifting on straights** — raise Ki to 0.001

**Too slow overall** — push lfSpeed to 240, raise start speed to 80

**Loses line on tight corners** — increase recovery spin from 80 to 110

---

## Project Structure
---

## Skills Demonstrated

- Embedded C++ on ESP32 (Xtensa dual-core, FreeRTOS)
- Discrete PID controller design and iterative hardware tuning
- FreeRTOS task creation and core pinning (xTaskCreatePinnedToCore)
- ESP32 LEDC PWM peripheral configuration
- WiFi AP mode and HTTP web server on embedded hardware
- 12-bit ADC interfacing and per-sensor normalisation
- TB6612FNG H-bridge motor driver control
- Volatile shared state between concurrent RTOS tasks

---

## About

I built this to get a deep understanding of closed-loop control on real constrained hardware. The interesting problems were figuring out why the robot oscillated at high speed, isolating the control loop from WiFi interference using dual-core task pinning, and dealing with ESP32's ADC2/WiFi hardware conflict. The live tuning dashboard made the whole iteration cycle significantly faster compared to reflashing for every parameter change.

If you're working on something similar or want to talk through the dual-core setup or PID tuning, feel free to open an issue or reach out.

---

*Part of my robotics and embedded systems portfolio — [github.com/Deepakk-06](https://github.com/Deepakk-06)*

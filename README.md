# PID Line Following Robot

> Autonomous ground robot with 8-channel IR sensing and a real-time discrete PID controller — built, tuned, and documented from scratch.

This is one of the projects from my personal robotics portfolio. It started as a standard line follower and turned into a proper study of PID control, sensor calibration, and embedded optimisation on AVR hardware. Everything here — firmware, tuning, calibration logic — is hand-written and tested on real hardware.

---

## How it works

The robot reads 8 IR sensors mounted in a row under the chassis. Each sensor is mapped to a positional weight. The weighted average of all active sensor readings gives a signed error value — positive means the line is to the left, negative means it's to the right, zero means the robot is dead centre.

```
Sensor  :  S1   S2   S3   S4   S5   S6   S7   S8
Weight  :  +4   +3   +2   +1   -1   -2   -3   -4
```

That error feeds into a discrete PID controller every loop iteration:

```
error      = weighted_sum(sensors) / active_sensors

PID_output = (Kp × error) + (Ki × ∫error) + (Kd × Δerror)

leftSpeed  = baseSpeed − PID_output
rightSpeed = baseSpeed + PID_output
```

The differential speed correction steers the robot back onto the line. More error = more aggressive correction. The derivative term (`Kd`) is what prevents overshoot and oscillation at high speed — it resists rapid changes in error, essentially predicting where the error is heading and counteracting it before it gets worse.

---

## Features

- 8-channel IR sensor array with per-sensor adaptive calibration
- Full PID loop — proportional, integral, and derivative terms
- Anti-windup clamp on the integrator (`±500`)
- Integrator resets to zero on line loss to prevent accumulated error
- Bidirectional calibration sweep — handles any lighting condition
- Smooth speed ramp from standstill to cruise (no wheel slip on start)
- Coast-to-stop motor behaviour — less mechanical stress at high speed
- Line-loss recovery using directional memory from last valid error
- Configurable for black-on-white or white-on-black tracks

---

## Hardware

| Component | Part |
|---|---|
| Microcontroller | Arduino Nano (ATmega328P) |
| Motor Driver | TB6612FNG Dual H-Bridge |
| Sensor Array | 8× IR analog sensors (A0–A7) |
| Drive Motors | 2× DC gear motors |
| Button 1 (Pin 11) | Trigger calibration |
| Button 2 (Pin 12) | Start run |
| LED (Pin 13) | On-line indicator |

---

## Wiring

```
TB6612FNG
  AIN1 → D4    AIN2 → D3    PWMA → D9
  BIN1 → D6    BIN2 → D7    PWMB → D10
  STBY → D5

IR Sensors  →  A0 to A7  (left to right across chassis)
Button 1    →  D11  (INPUT_PULLUP)
Button 2    →  D12  (INPUT_PULLUP)
LED         →  D13
```

---

## PID Parameters

| Gain | Value | Effect |
|---|---|---|
| `Kp` | `0.13` | Steering force proportional to current error |
| `Kd` | `0.40` | Damping — counteracts rapid error change, kills oscillation |
| `Ki` | `0.0005` | Eliminates residual steady-state drift on long straights |
| `lfSpeed` | `220` | Cruise speed (out of 255 PWM) |
| `currentSpeed` | `60` | Start speed — increments to `lfSpeed` each iteration |

---

## Running the robot

1. Flash `PID_LineFollower.ino` to the Arduino
2. Place the robot centred on the line
3. Press **Button 1** — the robot sweeps right then left (~10 seconds) to calibrate all 8 sensors against the current lighting
4. Press **Button 2** — it starts following

Open Serial Monitor at `115200` baud to see calibration thresholds printed per sensor. If any threshold looks wrong (near 0 or near 1023), that sensor likely didn't cross both the line and background during calibration — redo it.

---

## Tuning guide

**Robot oscillates (wobbles side to side)**
Raise `Kd` first — try `0.45`, then `0.50`. If that's not enough, pull `Kp` down slightly to `0.11`.

**Slow reaction on sharp corners**
Raise `Kp` to `0.15`. Also check `lfSpeed` isn't too fast for the track's corner radius.

**Drifts on straight sections**
Raise `Ki` slightly — `0.001` is a safe next step.

**Too slow overall**
Push `lfSpeed` toward `240`. Raise starting `currentSpeed` to `80`.

**Loses line on tight inside corners**
Increase the recovery spin value in `loop()` — currently `80`, try `100–120`.

---

## Calibration logic

Before every run the robot does a bidirectional sweep — spinning right for 5000 iterations, then left for 5000 more — while continuously updating the per-sensor min and max. The threshold for each sensor is `(min + max) / 2`.

This approach accounts for the fact that not all sensors have identical sensitivity and that IR reflectivity varies significantly between surfaces, print materials, and ambient lighting. A single global threshold would misclassify sensors at the edges of the array. Per-sensor thresholds eliminate that entirely.

---

## Firmware architecture

```
loop()
 ├── readLine()          normalise ADC → 0–1000, set sensorArray[]
 ├── lineFollow()        compute PID, set motor speeds
 │    ├── weighted error from sensorArray[]
 │    ├── P, I (clamped), D terms
 │    └── motor1run() / motor2run()
 └── recovery logic      spin toward last known error on line loss

calibrate()
 ├── updateCalibration() sample all sensors, track min/max
 └── compute threshold[] = (min + max) / 2 per sensor
```

---

## Repository

```
PID-Line-Following-Robot/
├── PID_LineFollower.ino    main Arduino sketch
└── README.md               this file
```

---

## Skills involved

- Embedded C++ on AVR (ATmega328P)
- Discrete PID controller design and manual tuning
- ADC prescaler optimisation for faster sensor sampling
- 8-channel analog sensor array interfacing and normalisation
- H-bridge motor driver control (direction + PWM)
- Iterative hardware debugging and parameter tuning on physical robot

---

## About this project

I built this to get a solid understanding of closed-loop control on constrained hardware. The interesting part wasn't the line following itself — it was figuring out why the robot oscillated at high speed, what role each PID term actually played, and how to make calibration reliable across different tracks and lighting setups.

If you're working on something similar and want to talk through tuning or sensor setup, feel free to open an issue or reach out.

---

*Part of my robotics and embedded systems portfolio — [github.com/Deepakk-06](https://github.com/Deepakk-06)*

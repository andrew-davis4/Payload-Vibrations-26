# Payload Vibrations — 2D Planar Bearing Dampening System

An Arduino Nano–based vibration dampening system for a rocket payload. The board uses an MPU6050 IMU to detect vibration and tilt in real time, applies Kalman filtering to clean the signal, and drives two SG90 servos through a PCA9685 PWM driver to actively counteract motion — keeping the payload stable during flight.

---

## How It Works

```
MPU6050 (IMU)
    └─▶ IMUManager        reads raw accelerometer + gyroscope data, applies Kalman filter
         └─▶ MotionController   maps filtered tilt angle → servo pulse using tunable gains
              └─▶ ServoController    sends PWM signal to PCA9685 → moves X and Y servos
```

The control strategy is **open-loop feed-forward** — no PID, no position feedback. Servo position is calculated directly from the filtered IMU output. All tuning constants (gains, deadband, servo limits) live in one place: `include/Config.h`.

---

## Project Structure

```
Payload-Vibrations-26/
├── include/
│   ├── Config.h               ← ALL tunable constants — start here
│   ├── Types.h                ← Shared data structs (IMUData, ServoCommand, etc.)
│   ├── Kalman.h / .cpp        ← Kalman filter (single axis)
│   ├── I2CHelper.h / .cpp     ← I2C bus read/write with timeout + recovery
│   ├── IMUManager.h / .cpp    ← MPU6050 init, read, filter
│   ├── ServoController.h/.cpp ← PCA9685 init, pulse output, clamping
│   ├── MotionController.h/.cpp← Feed-forward tilt → servo mapping
│   └── CalibrationManager.h/.cpp ← Startup bias calibration
└── src/
    └── main.cpp               ← setup() and loop() only
```

---

## Hardware on the PCB

| Component | Part | Interface |
|---|---|---|
| Microcontroller | Arduino Nano (ATmega328P) | — |
| IMU | MPU6050 | I2C @ 0x68 |
| PWM Driver | PCA9685 | I2C @ 0x40 |
| Servo X | SG90 | PCA9685 ch.15 |
| Servo Y | SG90 | PCA9685 ch.14 |

The PCB comes pre-assembled. You do not need to wire anything — just plug in USB and flash.

---

## Setting Up Your Development Environment

### 1. Install VS Code

1. Go to [https://code.visualstudio.com](https://code.visualstudio.com) and download the installer for your OS
2. Run the installer — default options are fine
3. Launch VS Code

### 2. Install the PlatformIO Extension

1. In VS Code, click the **Extensions** icon in the left sidebar (or press `Ctrl+Shift+X`)
2. Search for **PlatformIO IDE**
3. Click **Install** — this will also install Python dependencies automatically (takes a few minutes)
4. When it finishes, **restart VS Code** when prompted
5. You should now see a PlatformIO alien-head icon in the left sidebar

### 3. Install Git

If you don't already have Git installed:

1. Go to [https://git-scm.com/download/win](https://git-scm.com/download/win) (or mac/linux as appropriate)
2. Run the installer — default options are fine
3. Verify it worked by opening a terminal and running:
   ```
   git --version
   ```

### 4. Clone the Repository

1. Open VS Code
2. Press `Ctrl+Shift+P` to open the command palette
3. Type **Git: Clone** and select it
4. Paste the repository URL:
   ```
   https://github.com/andrew-davis4/Payload-Vibrations-26.git
   ```
5. Choose a folder on your machine to save it into
6. When VS Code asks **"Would you like to open the cloned repository?"** — click **Open**
7. PlatformIO will detect the `platformio.ini` and configure the project automatically

### 5. Build the Project

1. Click the **PlatformIO icon** in the left sidebar
2. Under **PROJECT TASKS → nanoatmega328 → General**, click **Build**
3. The first build will download the required libraries automatically (Adafruit PWM Servo Driver, Adafruit BusIO)
4. You should see `[SUCCESS]` in the terminal at the bottom

If you see errors about missing headers, make sure you opened the **root folder** of the cloned repo (the one containing `platformio.ini`), not a subfolder inside it.

---

## Flashing to the PCB

1. Plug the PCB into your computer via USB (Nano uses a **Mini-B USB** cable)
2. In PlatformIO sidebar → **Upload** (or press the **→ arrow** button in the bottom toolbar)
3. PlatformIO will compile and flash automatically
4. Open **Serial Monitor** (the plug icon in the bottom toolbar) at **115200 baud** to see live debug output

> **On startup the device runs a calibration routine — keep the PCB flat and still for ~1 second after plugging in.**

---

## Making and Testing Changes

### The only file you should need to edit for tuning:

**`include/Config.h`** — every tunable constant is here with comments explaining what it does:

| Constant | What it controls |
|---|---|
| `GainX` / `GainY` | How aggressively the servos respond to tilt |
| `DeadbandX` / `DeadbandY` | Minimum tilt before servos move (reduces jitter) |
| `ServoPulseMin/Max/Center` | Physical servo travel limits |
| `KalmanQAngle` / `RMeasure` | Filter smoothness vs. responsiveness |

### Workflow for a change

```
Edit Config.h  →  Build  →  Upload  →  Open Serial Monitor  →  Observe output
```

### Saving your changes back to GitHub

After making and testing a change, push it so the team can see it:

```bash
git add .
git commit -m "describe what you changed and why"
git push
```

Or use the VS Code Source Control panel (`Ctrl+Shift+G`) — stage changes with **+**, write a message, click **Commit**, then **Sync Changes**.

---

## Serial Monitor Debug Output

When `DebugEnabled = true` in `Config.h`, the serial monitor prints live data every 100ms:

```
filtX: 0.0023    filtY: -0.0011    pwmX: 348    pwmY: 352
```

- **filtX / filtY** — Kalman-filtered tilt angle output (should be near 0 when flat and still)
- **pwmX / pwmY** — servo pulse counts being sent to PCA9685 (center = 350)

To disable debug output for a clean build, set `DebugEnabled = false` in `Config.h`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `[FATAL] IMU init failed` | MPU6050 not found on I2C | Check USB connection; confirm PCB is powered |
| `[FATAL] Servo driver init failed` | PCA9685 not found on I2C | Same as above |
| Servos jitter constantly at rest | Deadband too small | Increase `DeadbandX`/`DeadbandY` in Config.h |
| Servos barely move | Gain too low | Increase `GainX`/`GainY` in Config.h |
| Servos slam to limit | Gain too high | Decrease `GainX`/`GainY` in Config.h |
| Build fails on fresh clone | Wrong folder opened | Make sure `platformio.ini` is in the root of the opened folder |

---

## Dependencies

Managed automatically by PlatformIO — no manual installation needed:

- [Adafruit PWM Servo Driver Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO) (pulled in automatically)
- Wire (Arduino built-in)

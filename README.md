# Payload Vibrations — 2D Planar Bearing Dampening System

A vibration dampening system for July 2026 QRET rocket payload based on Arduino Nano. 

The board uses an onboard MPU6050 IMU to detect acceleration and tilt changes in real time, applies Kalman filtering to clean the signal, and drives two SG90 servos through a PCA9685 PWM driver to actively counteract motion — keeping the payload stable during flight.

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
|   ├── CalibrationManager.h        ← Startup calibration
│   ├── Config.h                    ← ALL constants 
│   ├── I2CHelper.h                 ← I2C bus read/write + timeout recovery
│   ├── IMUManager.h                ← MPU6050 init, read, filter
│   ├── Kalman.h                    ← Single axis Kalman filter library
│   ├── MotionController.h          ← Feed-forward tilt and servo mapping
│   ├── ServoController.h           ← PCA9685 init, PWM pulse output, clamping
│   └── Types.h                     ← Custom structs
└── src/
    ├── CalibrationManager.cpp
    ├── I2CHelper.cpp
    ├── IMUManager.cpp
    ├── Kalman.cpp                  ← Kalman filter source code
    ├── main.cpp                    ← setup() and loop() only
    ├── MotionController.cpp
    └── ServoController.cpp
```

---

## Hardware on the PCB

| Component | Part | Interface |
|---|---|---|
| Microcontroller | Arduino Nano (ATmega328P) | — |
| IMU | MPU6050 | I2C @ 0x68 |
| PWM Driver | PCA9685 | I2C @ 0x40 |
| Servo X1 | SG90 | PCA9685 ch.15 |
| Servo X2 | SG90 | PCA9685 ch.14 |
| Servo Y1 | SG90 | PCA9685 ch.13 |
| Servo Y2 | SG90 | PCA9685 ch.12 |

The PCB is pre-assembled and you do not need to wire anything. Just plug in USB and use the IDE instructions below to flash/test new code.

---

## Steps to set up the Dev environment to test the board

### 1. Install VS Code

1. Download from [https://code.visualstudio.com](https://code.visualstudio.com)
2. Run the installer with default options
3. Launch VS Code

### 2. Install the PlatformIO Extension

1. In VS Code, click the **Extensions** icon in the left sidebar (or press `Ctrl+Shift+X`)
2. Search for **PlatformIO IDE**
3. Click **Install**
4. When it finishes, **restart VS Code** when prompted
5. You should now see PlatformIO (alien-head icon) in the left sidebar

### 3. (Optional) Install Git (For making software changes)

If you don't already have Git installed:

1. Download from [https://git-scm.com/download/win](https://git-scm.com/download/win)
2. Run the installer, with default options
3. Verify it works by opening a git bash terminal and running:
   ```
   git --version
   ```

### 4. (Optional) Clone the Repository

1. Open VS Code
2. Press `Ctrl+Shift+P` to open the command palette
3. Type **Git: Clone** and select it
4. Paste the repository URL:
   ```
   https://github.com/andrew-davis4/Payload-Vibrations-26.git
   ```
5. Choose a folder on your machine to save it into (Some local projects folder for QRET)
6. When VS Code asks **"Would you like to open the cloned repository?"**, click **Open**
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

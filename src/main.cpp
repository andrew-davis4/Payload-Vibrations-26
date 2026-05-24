#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Config.h"
#include "Types.h"
#include "I2CHelper.h"
#include "IMUManager.h"
#include "ServoController.h"
#include "MotionController.h"
#include "CalibrationManager.h"

static IMUManager         imu;
static ServoController    servos;
static MotionController   controller;
static CalibrationManager calibration;

// Optional debug print throttle
static uint32_t _lastDebugPrintMs = 0;

// =============================================================================
// setup()
// =============================================================================
void setup()
{
    Serial.begin(Config::SerialBaudRate);

    if (Config::DebugEnabled)
    {
        Serial.println(F("=== PlanarBearingController ==="));
        Serial.println(F("Initializing..."));
    }

    // 1. Initialize I2C bus
    I2CHelper::init();

    // 2. Initialize PCA9685 servo driver (centers servos on boot)
    if (!servos.init())
    {
        Serial.println(F("[FATAL] Servo driver init failed - halting"));
        while (true); // Halt; check wiring and I2C address
    }

    // 3. Initialize MPU6050
    if (!imu.init())
    {
        Serial.println(F("[FATAL] IMU init failed - halting"));
        while (true); // Halt; check wiring and I2C address
    }

    // 4. Run startup calibration
    //    Keep device FLAT and STILL during this phase.
    calibration.run(imu);

    if (Config::DebugEnabled)
    {
        Serial.println(F("Initialization complete. Entering main loop."));
    }
}

// =============================================================================
// loop()
// =============================================================================
void loop()
{
    // 1. Read and filter IMU data
    IMUData imuData = imu.update();

    // 2. Compute servo commands (open-loop feed-forward)
    //    Target setpoints are 0 (flat/level) - adjust if non-zero bias desired
    ServoCommand cmd = controller.compute(imuData, 0.0f, 0.0f);

    // 3. Apply servo outputs
    servos.apply(cmd);

    // 4. Optional debug output (rate-limited to avoid flooding serial)
    if (Config::DebugEnabled)
    {
        uint32_t now = millis();
        if (now - _lastDebugPrintMs >= Config::DebugPrintIntervalMs)
        {
            _lastDebugPrintMs = now;
            if (imuData.valid)
            {
                Serial.print(F("filtX: ")); Serial.print(imuData.filtX, 4);
                Serial.print(F("\tfiltY: ")); Serial.print(imuData.filtY, 4);
                Serial.print(F("\tpwmX: ")); Serial.print(cmd.pulseX);
                Serial.print(F("\tpwmY: ")); Serial.println(cmd.pulseY);
            }
        }
    }
}

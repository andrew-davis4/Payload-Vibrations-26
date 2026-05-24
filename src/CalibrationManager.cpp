// =============================================================================
// CalibrationManager.cpp
// Startup calibration: gyro bias estimation and flat-surface accel offsets.
//
// Procedure:
//   1. Place the device on a flat, stationary surface.
//   2. CalibrationManager::run() collects Config::CalibrationSamples raw reads.
//   3. Averages gyro readings -> gyro bias.
//   4. Averages accel X/Y -> accel offset (expected 0g on flat surface).
//   5. Derives initial Kalman seed angles from averaged accel values.
//   6. Pushes results into IMUManager via applyCalibration().
//
// NOTE: The device must be stationary and level during calibration.
// =============================================================================

#include "CalibrationManager.h"
#include "I2CHelper.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

CalibrationManager::CalibrationManager()
{
}

void CalibrationManager::run(IMUManager& imu)
{
    if (Config::DebugEnabled)
    {
        Serial.println(F("[Cal] Starting calibration - hold device flat and still..."));
    }

    // Accumulators for averaging (use double to preserve precision over many samples)
    double sumGX = 0.0, sumGY = 0.0, sumGZ = 0.0;
    double sumAX = 0.0, sumAY = 0.0, sumAZ = 0.0;

    uint16_t validSamples = 0;

    for (uint16_t i = 0; i < Config::CalibrationSamples; i++)
    {
        IMUData sample = imu.update();

        if (sample.valid)
        {
            sumGX += sample.rawGyroX;
            sumGY += sample.rawGyroY;
            sumGZ += sample.rawGyroZ;

            sumAX += sample.rawAccX;
            sumAY += sample.rawAccY;
            sumAZ += sample.rawAccZ;

            validSamples++;
        }

        delay(Config::CalibrationDelayMs);
    }

    if (validSamples == 0)
    {
        if (Config::DebugEnabled)
        {
            Serial.println(F("[Cal] FAILED - no valid IMU samples received"));
        }
        return;
    }

    const float n = static_cast<float>(validSamples);

    // --- Gyro bias: average zero-rate readings ---
    _calData.gyroBiasX = static_cast<float>(sumGX / n);
    _calData.gyroBiasY = static_cast<float>(sumGY / n);
    _calData.gyroBiasZ = static_cast<float>(sumGZ / n);

    // --- Accel offset: deviation from expected flat-surface reading ---
    // On a flat surface, ideal readings are: X=0g, Y=0g, Z=1g.
    // Store the X and Y averages directly as offsets; Z is not used for control.
    _calData.accelOffsetX = static_cast<float>(sumAX / n);
    _calData.accelOffsetY = static_cast<float>(sumAY / n);
    _calData.accelOffsetZ = static_cast<float>(sumAZ / n) - 1.0f; // Remove gravity

    // --- Seed angles: derive from averaged accel values ---
    // After removing offset, both should be ~0 on a flat surface.
    // Using the raw averages before offset subtraction to get the actual tilt angle.
    // TODO: For true angle estimation, use atan2(accY, sqrt(accX^2+accZ^2)).
    //       This simplified version seeds to 0 assuming startup is level.
    _calData.initAngleX = 0.0f;
    _calData.initAngleY = 0.0f;

    _calData.ready = true;

    // Push calibration data into the IMU manager
    imu.applyCalibration(_calData);

    if (Config::DebugEnabled)
    {
        Serial.println(F("[Cal] Calibration complete."));
        Serial.print(F("[Cal] Gyro bias  X/Y/Z: "));
        Serial.print(_calData.gyroBiasX, 4); Serial.print(F(" / "));
        Serial.print(_calData.gyroBiasY, 4); Serial.print(F(" / "));
        Serial.println(_calData.gyroBiasZ, 4);
        Serial.print(F("[Cal] Accel off  X/Y/Z: "));
        Serial.print(_calData.accelOffsetX, 4); Serial.print(F(" / "));
        Serial.print(_calData.accelOffsetY, 4); Serial.print(F(" / "));
        Serial.println(_calData.accelOffsetZ, 4);
    }
}

const CalibrationData& CalibrationManager::getCalibrationData() const
{
    return _calData;
}

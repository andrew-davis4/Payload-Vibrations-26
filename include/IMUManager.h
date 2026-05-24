#pragma once

#include "Types.h"
#include "Kalman.h"
#include <stdint.h>

// =============================================================================
// IMUManager.h
// Owns the MPU6050 sensor, Kalman filters, and all IMU timing state.
//
// Responsibilities:
//   - MPU6050 register initialization
//   - Raw sensor burst-read (14 bytes: accel + temp + gyro)
//   - Sensor scaling (raw ADC -> physical units)
//   - dt computation via micros()
//   - Kalman filter application
//   - Calibration offset application
//
// Does NOT:
//   - Control servos
//   - Contain motion mapping logic
//   - Contain application-level decisions
// =============================================================================

class IMUManager
{
public:
    IMUManager();

    // Initialize the MPU6050. Verifies WHO_AM_I register.
    // Returns true on success, false if the sensor is not responding.
    bool init();

    // Read raw sensor data, apply calibration offsets, run Kalman filters,
    // compute dt, and return a fully populated IMUData struct.
    // Call this once per main loop iteration.
    IMUData update();

    // Apply calibration offsets and seed the Kalman filter initial angles.
    // Call this after CalibrationManager::run() has produced CalibrationData.
    void applyCalibration(const CalibrationData& cal);

    // Seed Kalman filter initial angles directly (used during calibration).
    void seedAngles(float angleX, float angleY);

private:
    Kalman  _kalmanX;
    Kalman  _kalmanY;

    uint32_t _lastTimeUs;  // micros() timestamp of last update
    uint8_t  _i2cBuf[14]; // Raw burst-read buffer (accel + temp + gyro)

    // Active calibration offsets
    float _gyroBiasX;
    float _gyroBiasY;
    float _accelOffsetX;
    float _accelOffsetY;

    bool _calibrated;
};

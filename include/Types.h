#pragma once

#include <stdint.h>

// =============================================================================
// Types.h
// Shared lightweight POD structs used across modules.
// No dynamic allocation. No virtual functions. No STL.
// =============================================================================

// -----------------------------------------------------------------------------
// IMUData
// Output of IMUManager::update(). Contains both raw and filtered sensor values,
// plus the elapsed time delta used for integration.
// -----------------------------------------------------------------------------
struct IMUData
{
    // Raw accelerometer readings (in g, after scale factor applied)
    float rawAccX = 0.0f;
    float rawAccY = 0.0f;
    float rawAccZ = 0.0f;

    // Raw gyroscope readings (in deg/s, after scale factor applied)
    float rawGyroX = 0.0f;
    float rawGyroY = 0.0f;
    float rawGyroZ = 0.0f;

    // Kalman-filtered outputs (used by MotionController)
    float filtX = 0.0f;
    float filtY = 0.0f;

    // Elapsed time since last update (seconds)
    float dt = 0.0f;

    // True if this reading is valid (I2C succeeded)
    bool valid = false;
};

// -----------------------------------------------------------------------------
// ServoCommand
// Encapsulates final PWM pulse outputs for both servo axes.
// Consumed by ServoController::apply().
// -----------------------------------------------------------------------------
struct ServoCommand
{
    uint16_t pulseX = 350; // PCA9685 12-bit count for X servo
    uint16_t pulseY = 350; // PCA9685 12-bit count for Y servo
};

// -----------------------------------------------------------------------------
// CalibrationData
// Holds sensor offsets and biases computed during the startup calibration run.
// Stored and applied by CalibrationManager; read by IMUManager.
// -----------------------------------------------------------------------------
struct CalibrationData
{
    // Gyroscope zero-rate bias (deg/s)
    float gyroBiasX = 0.0f;
    float gyroBiasY = 0.0f;
    float gyroBiasZ = 0.0f;

    // Accelerometer offset (g) relative to flat/level reference
    float accelOffsetX = 0.0f;
    float accelOffsetY = 0.0f;
    float accelOffsetZ = 0.0f;

    // Initial Kalman seed angles (deg) - derived from accel at rest
    float initAngleX = 0.0f;
    float initAngleY = 0.0f;

    bool ready = false; // Set to true once calibration has been run
};

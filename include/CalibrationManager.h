#pragma once

#include "Types.h"
#include "IMUManager.h"

// =============================================================================
// CalibrationManager.h
// Startup calibration routine for flat-surface bias estimation.
//
// Responsibilities:
//   - Collect multiple raw IMU samples while device is stationary and flat
//   - Average samples to estimate gyro zero-rate bias
//   - Estimate accelerometer offset relative to gravity vector
//   - Derive initial Kalman seed angles from accel data
//   - Store results in a CalibrationData struct
//   - Push calibration results into IMUManager
//
// Usage:
//   CalibrationManager cal;
//   cal.run(imu);   // Blocks for CalibrationSamples * CalibrationDelayMs ms
// =============================================================================

class CalibrationManager
{
public:
    CalibrationManager();

    // Run the full calibration routine. Blocks until complete.
    // Reads raw IMU data via imu, computes offsets, and calls imu.applyCalibration().
    void run(IMUManager& imu);

    // Access the calibration results after run() completes.
    const CalibrationData& getCalibrationData() const;

private:
    CalibrationData _calData;
};

#pragma once

#include "Types.h"

// =============================================================================
// MotionController.h
// Open-loop feed-forward motion mapping from filtered IMU data to servo pulses.
//
// Responsibilities:
//   - Apply axis gains (Config::GainX / GainY)
//   - Apply deadband filtering to suppress jitter
//   - Map filtered IMU values to PCA9685 pulse counts
//   - Return a ServoCommand ready for ServoController::apply()
//
// Architecture note:
//   This module intentionally contains only feed-forward logic. The compute()
//   interface accepts target setpoints so that a future closed-loop (PID)
//   upgrade can be dropped in without restructuring the rest of the system.
// =============================================================================

class MotionController
{
public:
    MotionController();

    // Compute servo commands from current filtered IMU state.
    //
    // imuData   - output of IMUManager::update()
    // targetX   - desired X position setpoint (0.0 = center / flat)
    // targetY   - desired Y position setpoint (0.0 = center / flat)
    //
    // Returns a ServoCommand with clamped pulse values for both axes.
    ServoCommand compute(const IMUData& imuData,
                         float targetX = 0.0f,
                         float targetY = 0.0f);

private:
    // Apply deadband: zero the signal if its absolute value is below threshold.
    float applyDeadband(float value, float threshold) const;

    // Map a scalar error value to a servo pulse count around center.
    uint16_t mapToPulse(float error, float gain, uint16_t center) const;
};

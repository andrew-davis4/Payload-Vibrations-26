// =============================================================================
// MotionController.cpp
// Open-loop feed-forward mapping: filtered IMU -> ServoCommand.
//
// Control philosophy:
//   error  = filteredValue - targetSetpoint
//   pulse  = centerPulse - (error * gain)
//
// The subtraction sign means a positive tilt in X causes the X servo to
// move in the compensating direction. Flip the sign in mapToPulse() or
// negate the gain in Config if your mechanism requires the opposite polarity.
//
// TODO: Tune Config::GainX / GainY and Config::DeadbandX / DeadbandY
//       on the physical rig to achieve desired response.
// =============================================================================

#include "MotionController.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

MotionController::MotionController()
{
}

ServoCommand MotionController::compute(const IMUData& imuData,
                                        float targetX,
                                        float targetY)
{
    ServoCommand cmd;

    if (!imuData.valid)
    {
        // IMU read failed - hold center position as a safe fallback
        cmd.pulseX = Config::ServoPulseCenter;
        cmd.pulseY = Config::ServoPulseCenter;
        return cmd;
    }

    // Compute errors relative to setpoints
    float errorX = imuData.filtX - targetX;
    float errorY = imuData.filtY - targetY;

    // Apply deadband filtering to suppress jitter near neutral
    errorX = applyDeadband(errorX, Config::DeadbandX);
    errorY = applyDeadband(errorY, Config::DeadbandY);

    // Map errors to servo pulse outputs
    cmd.pulseX = mapToPulse(errorX, Config::GainX, Config::ServoPulseCenter);
    cmd.pulseY = mapToPulse(errorY, Config::GainY, Config::ServoPulseCenter);

    return cmd;
}

float MotionController::applyDeadband(float value, float threshold) const
{
    if (fabsf(value) < threshold)
    {
        return 0.0f;
    }
    return value;
}

uint16_t MotionController::mapToPulse(float error, float gain, uint16_t center) const
{
    // Pulse = center - (error * gain)
    // A positive error drives the servo below center; negative drives it above.
    // The constrain call is a final safety clamp; ServoController also clamps.
    float pulse = static_cast<float>(center) - (error * gain);

    // Clamp to valid pulse range (int cast is safe after clamping)
    if (pulse < static_cast<float>(Config::ServoPulseMin))
        pulse = static_cast<float>(Config::ServoPulseMin);
    if (pulse > static_cast<float>(Config::ServoPulseMax))
        pulse = static_cast<float>(Config::ServoPulseMax);

    return static_cast<uint16_t>(pulse);
}

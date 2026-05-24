#pragma once

#include "Types.h"
#include <Adafruit_PWMServoDriver.h>

// =============================================================================
// ServoController.h
// Owns the PCA9685 PWM driver and all servo output logic.
//
// Responsibilities:
//   - PCA9685 initialization
//   - Servo pulse output (both axes)
//   - Pulse clamping to configured safety limits
//   - Centering helper
//
// Does NOT:
//   - Perform motion calculations
//   - Read from IMU
//   - Contain control logic
// =============================================================================

class ServoController
{
public:
    ServoController();

    // Initialize the PCA9685 driver and set PWM frequency.
    // Returns true on success.
    bool init();

    // Apply a ServoCommand: clamp pulses to safe limits and output to PCA9685.
    void apply(const ServoCommand& cmd);

    // Drive both servos to the configured center pulse position.
    void center();

private:
    Adafruit_PWMServoDriver _pwm;

    // Clamp a pulse value to [ServoPulseMin, ServoPulseMax]
    uint16_t clampPulse(uint16_t pulse) const;
};

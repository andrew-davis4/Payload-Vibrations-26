// =============================================================================
// ServoController.cpp
// PCA9685 initialization and servo pulse output.
// =============================================================================

#include "ServoController.h"
#include "Config.h"
#include <Arduino.h>

ServoController::ServoController()
    : _pwm()
{
}

bool ServoController::init()
{
    _pwm.begin();
    _pwm.setPWMFreq(Config::ServoPWMFreqHz);

    // Drive to center on startup so the mechanism is in a known position
    center();

    if (Config::DebugEnabled)
    {
        Serial.println(F("[Servo] PCA9685 initialized OK"));
    }
    return true;
}

void ServoController::apply(const ServoCommand& cmd)
{
    _pwm.setPWM(Config::ServoXChannel, 0, clampPulse(cmd.pulseX));
    _pwm.setPWM(Config::ServoYChannel, 0, clampPulse(cmd.pulseY));
}

void ServoController::center()
{
    ServoCommand centered;
    centered.pulseX = Config::ServoPulseCenter;
    centered.pulseY = Config::ServoPulseCenter;
    apply(centered);
}

uint16_t ServoController::clampPulse(uint16_t pulse) const
{
    if (pulse < Config::ServoPulseMin) return Config::ServoPulseMin;
    if (pulse > Config::ServoPulseMax) return Config::ServoPulseMax;
    return pulse;
}

// =============================================================================
// I2CHelper.cpp
// Low-level I2C utilities with timeout and bus recovery.
//
// Adapted from the project's custom I2C implementation (I2C.ino).
// All original robustness behaviors are preserved:
//   - Per-byte receive timeout via micros()
//   - Incomplete read detection (received < requested)
//   - Wire buffer drain on partial read
//   - Bus recovery (Wire.end/begin) on hardware error code 4
// =============================================================================

#include "I2CHelper.h"
#include "Config.h"
#include <Wire.h>
#include <Arduino.h>

namespace I2CHelper
{
    void init()
    {
        Wire.begin();
        Wire.setClock(Config::I2CClockHz);
    }

    void recoverBus()
    {
        if (Config::DebugEnabled)
        {
            Serial.println(F("[I2C] Bus error detected - attempting recovery..."));
        }

        Wire.end();
        delay(10);
        Wire.begin();
        Wire.setClock(Config::I2CClockHz);

        if (Config::DebugEnabled)
        {
            Serial.println(F("[I2C] Bus recovery complete."));
        }
    }

    uint8_t write(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop)
    {
        Wire.beginTransmission(Config::IMUAddress);
        Wire.write(registerAddress);
        Wire.write(data, length);
        uint8_t rcode = Wire.endTransmission(sendStop);

        if (rcode)
        {
            if (Config::DebugEnabled)
            {
                Serial.print(F("[I2C] Write failed, code: "));
                Serial.println(rcode);
            }
            if (rcode == 4)
            {
                recoverBus();
            }
        }
        return rcode;
    }

    uint8_t write(uint8_t registerAddress, uint8_t data, bool sendStop)
    {
        return write(registerAddress, &data, 1, sendStop);
    }

    uint8_t read(uint8_t registerAddress, uint8_t* data, uint8_t nbytes)
    {
        // --- Address phase ---
        Wire.beginTransmission(Config::IMUAddress);
        Wire.write(registerAddress);
        uint8_t rcode = Wire.endTransmission(false); // Repeated start - don't release bus

        if (rcode)
        {
            if (Config::DebugEnabled)
            {
                Serial.print(F("[I2C] Read address phase failed, code: "));
                Serial.println(rcode);
            }
            if (rcode == 4)
            {
                recoverBus();
            }
            return rcode;
        }

        // --- Data phase ---
        uint8_t received = Wire.requestFrom(Config::IMUAddress, nbytes, (uint8_t)true);

        // If we didn't immediately get all bytes, wait with timeout
        if (received < nbytes)
        {
            uint32_t timeoutStart = micros();
            while ((Wire.available() < nbytes) &&
                   ((micros() - timeoutStart) < Config::I2CTimeoutUs))
            {
                // Busy-wait with timeout
            }
        }

        if (Wire.available() >= nbytes)
        {
            for (uint8_t i = 0; i < nbytes; i++)
            {
                data[i] = Wire.read();
            }
            return 0; // Success
        }
        else
        {
            if (Config::DebugEnabled)
            {
                Serial.println(F("[I2C] Read timeout / incomplete"));
            }

            // Drain any partial data to prevent buffer offset on next read
            while (Wire.available())
            {
                Wire.read();
            }
            return 5; // Custom timeout error code (not used by endTransmission)
        }
    }

} // namespace I2CHelper

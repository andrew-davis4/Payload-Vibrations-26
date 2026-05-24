#pragma once

#include <stdint.h>

// =============================================================================
// I2CHelper.h
// Low-level I2C utility functions with timeout handling and bus recovery.
//
// Adapted from original custom I2C implementation; preserves all robustness
// behavior including:
//   - Per-byte receive timeout
//   - Incomplete read detection
//   - Wire buffer drain on error
//   - Hardware error (code 4) bus recovery via Wire.end()/begin()
// =============================================================================

namespace I2CHelper
{
    // Initialize the I2C bus at the configured clock speed.
    void init();

    // Write one byte to a register on the IMU.
    // registerAddress - target register
    // data            - single byte value
    // sendStop        - whether to release the bus after transmission
    // Returns 0 on success, Wire error code otherwise.
    uint8_t write(uint8_t registerAddress, uint8_t data, bool sendStop);

    // Write multiple bytes starting at registerAddress.
    // Returns 0 on success, Wire error code otherwise.
    uint8_t write(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop);

    // Read nbytes from registerAddress into data buffer.
    // Returns 0 on success, 5 on timeout/incomplete read, Wire error code otherwise.
    uint8_t read(uint8_t registerAddress, uint8_t* data, uint8_t nbytes);

    // Attempt to recover the I2C bus after a hardware error.
    // Performs Wire.end() / Wire.begin() / setClock().
    void recoverBus();
}

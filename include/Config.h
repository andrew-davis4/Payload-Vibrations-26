#pragma once

#include <stdint.h>

// =============================================================================
// Config.h
// Centralized compile-time configuration for PlanarBearingController.
// All tunable constants live here. Adjust values experimentally as needed.
// =============================================================================

namespace Config
{
    // -------------------------------------------------------------------------
    // Serial
    // -------------------------------------------------------------------------
    constexpr uint32_t SerialBaudRate = 115200;

    // -------------------------------------------------------------------------
    // Debug
    // Set to 1 to enable serial debug printing; 0 to disable entirely.
    // -------------------------------------------------------------------------
    constexpr bool DebugEnabled     = true;
    constexpr uint32_t DebugPrintIntervalMs = 100; // How often to print IMU data

    // -------------------------------------------------------------------------
    // I2C
    // -------------------------------------------------------------------------
    constexpr uint32_t I2CClockHz   = 400000UL;
    constexpr uint8_t  IMUAddress   = 0x68;        // MPU6050, AD0 low
    constexpr uint16_t I2CTimeoutUs = 1000;        // Per-byte timeout in microseconds

    // -------------------------------------------------------------------------
    // MPU6050 Registers
    // -------------------------------------------------------------------------
    constexpr uint8_t MPU_REG_SMPLRT_DIV  = 0x19;
    constexpr uint8_t MPU_REG_CONFIG      = 0x1A;
    constexpr uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t MPU_REG_ACCEL_CONFIG= 0x1C;
    constexpr uint8_t MPU_REG_ACCEL_XOUT  = 0x3B; // Burst-read start (14 bytes)
    constexpr uint8_t MPU_REG_PWR_MGMT_1  = 0x6B;
    constexpr uint8_t MPU_REG_WHO_AM_I    = 0x75;
    constexpr uint8_t MPU_WHO_AM_I_VALUE  = 0x68;

    // MPU6050 scale factors (±2g accel, ±250 deg/s gyro)
    constexpr float AccelScaleFactor = 16384.0f; // LSB/g for ±2g
    constexpr float GyroScaleFactor  = 131.0f;   // LSB/(deg/s) for ±250 deg/s

    // -------------------------------------------------------------------------
    // Calibration
    // -------------------------------------------------------------------------
    constexpr uint16_t CalibrationSamples   = 200;   // Samples averaged at startup
    constexpr uint32_t CalibrationDelayMs   = 5;     // Delay between calibration samples

    // -------------------------------------------------------------------------
    // Kalman Filter Tuning
    // TODO: Tune Q_angle, Q_bias, R_measure for your specific hardware noise.
    // -------------------------------------------------------------------------
    constexpr float KalmanQAngle   = 0.001f;  // Process noise - accelerometer
    constexpr float KalmanQBias    = 0.003f;  // Process noise - gyro bias
    constexpr float KalmanRMeasure = 0.03f;   // Measurement noise

    // -------------------------------------------------------------------------
    // PCA9685 / Servo Driver
    // -------------------------------------------------------------------------
    constexpr uint8_t  PCA9685Address   = 0x40;
    constexpr uint16_t ServoPWMFreqHz   = 60;

    // -------------------------------------------------------------------------
    // Servo Channels
    // -------------------------------------------------------------------------
    constexpr uint8_t ServoXChannel = 15;
    constexpr uint8_t ServoYChannel = 14;

    // -------------------------------------------------------------------------
    // Servo Pulse Limits (PCA9685 12-bit counts at 60 Hz)
    // TODO: Verify these limits against your physical servo travel.
    // Typical SG90: ~150 (0°) to ~600 (180°), center ~375
    // -------------------------------------------------------------------------
    constexpr uint16_t ServoPulseMin    = 150;
    constexpr uint16_t ServoPulseMax    = 600;
    constexpr uint16_t ServoPulseCenter = 350; // Neutral/center position

    // -------------------------------------------------------------------------
    // Motion Controller Gains
    // TODO: Tune these experimentally on the physical rig.
    // Gain maps filtered IMU value -> servo pulse offset.
    // -------------------------------------------------------------------------
    constexpr float GainX = 300.0f; // X-axis feed-forward gain
    constexpr float GainY = 300.0f; // Y-axis feed-forward gain

    // -------------------------------------------------------------------------
    // Deadband
    // Filtered values with absolute magnitude below this threshold are zeroed
    // to prevent servo jitter/chatter from sensor noise.
    // TODO: Tune to the noise floor of your filtered signal.
    // -------------------------------------------------------------------------
    constexpr float DeadbandX = 0.01f; // In same units as filtered IMU output
    constexpr float DeadbandY = 0.01f;

    // -------------------------------------------------------------------------
    // Update Rate
    // -------------------------------------------------------------------------
    constexpr uint32_t LoopIntervalUs = 0; // 0 = run as fast as possible
}

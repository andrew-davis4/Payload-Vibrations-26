// =============================================================================
// IMUManager.cpp
// MPU6050 initialization, burst-read, sensor scaling, and Kalman filtering.
// =============================================================================

#include "IMUManager.h"
#include "I2CHelper.h"
#include "Config.h"
#include <Arduino.h>
#include <math.h>

IMUManager::IMUManager()
    : _lastTimeUs(0)
    , _gyroBiasX(0.0f)
    , _gyroBiasY(0.0f)
    , _accelOffsetX(0.0f)
    , _accelOffsetY(0.0f)
    , _calibrated(false)
{
    // Zero the I2C buffer
    for (uint8_t i = 0; i < sizeof(_i2cBuf); i++)
    {
        _i2cBuf[i] = 0;
    }
}

bool IMUManager::init()
{
    // Configure MPU6050:
    //   0x19 - Sample rate divider: 7  -> 1 kHz / (1+7) = 125 Hz
    //   0x1A - DLPF config: 0x00       -> no digital low-pass filter
    //   0x1B - Gyro config:  0x11      -> ±250 deg/s full scale (bit layout: XG_ST|YG_ST|ZG_ST|FS_SEL[1:0]|-)
    //   0x1C - Accel config: 0x11      -> ±2g full scale
    // TODO: Adjust DLPF (0x1A) settings to suit your vibration environment.
    uint8_t cfg[4] = { 7, 0x00, 0x00, 0x00 };
    I2CHelper::write(Config::MPU_REG_SMPLRT_DIV, cfg, 4, false);

    // Wake MPU6050, select PLL with X-gyro as clock source
    I2CHelper::write(Config::MPU_REG_PWR_MGMT_1, 0x01, true);

    delay(100); // Allow sensor to stabilize

    // Verify identity
    if (I2CHelper::read(Config::MPU_REG_WHO_AM_I, _i2cBuf, 1) != 0)
    {
        if (Config::DebugEnabled)
        {
            Serial.println(F("[IMU] WHO_AM_I read failed"));
        }
        return false;
    }

    if (_i2cBuf[0] != Config::MPU_WHO_AM_I_VALUE)
    {
        if (Config::DebugEnabled)
        {
            Serial.print(F("[IMU] Unexpected WHO_AM_I: 0x"));
            Serial.println(_i2cBuf[0], HEX);
        }
        return false;
    }

    // Apply Kalman filter tuning from Config
    _kalmanX.setQangle(Config::KalmanQAngle);
    _kalmanX.setQbias(Config::KalmanQBias);
    _kalmanX.setRmeasure(Config::KalmanRMeasure);

    _kalmanY.setQangle(Config::KalmanQAngle);
    _kalmanY.setQbias(Config::KalmanQBias);
    _kalmanY.setRmeasure(Config::KalmanRMeasure);

    // Seed angles at 0 (flat); calibration will refine if run afterward
    _kalmanX.setAngle(0.0f);
    _kalmanY.setAngle(0.0f);

    _lastTimeUs = micros();

    if (Config::DebugEnabled)
    {
        Serial.println(F("[IMU] MPU6050 initialized OK"));
    }
    return true;
}

void IMUManager::applyCalibration(const CalibrationData& cal)
{
    _gyroBiasX    = cal.gyroBiasX;
    _gyroBiasY    = cal.gyroBiasY;
    _accelOffsetX = cal.accelOffsetX;
    _accelOffsetY = cal.accelOffsetY;
    _calibrated   = true;

    // Seed Kalman filters with the calibration-derived initial angles
    _kalmanX.setAngle(cal.initAngleX);
    _kalmanY.setAngle(cal.initAngleY);
}

void IMUManager::seedAngles(float angleX, float angleY)
{
    _kalmanX.setAngle(angleX);
    _kalmanY.setAngle(angleY);
}

IMUData IMUManager::update()
{
    IMUData data;

    // Compute dt
    uint32_t now = micros();
    data.dt = static_cast<float>(now - _lastTimeUs) / 1000000.0f;
    _lastTimeUs = now;

    // Guard against zero or absurdly large dt (e.g. first call, or overflow edge case)
    if (data.dt <= 0.0f || data.dt > 1.0f)
    {
        data.dt = 0.004f; // Fall back to 250 Hz assumption
    }

    // Burst-read 14 bytes: ACCEL_XOUT_H through GYRO_ZOUT_L
    if (I2CHelper::read(Config::MPU_REG_ACCEL_XOUT, _i2cBuf, 14) != 0)
    {
        // I2C read failed - return invalid data, keep last filter state
        data.valid = false;
        return data;
    }

    // --- Parse raw sensor values (big-endian 16-bit signed) ---
    int16_t rawAX = static_cast<int16_t>((_i2cBuf[0]  << 8) | _i2cBuf[1]);
    int16_t rawAY = static_cast<int16_t>((_i2cBuf[2]  << 8) | _i2cBuf[3]);
    int16_t rawAZ = static_cast<int16_t>((_i2cBuf[4]  << 8) | _i2cBuf[5]);
    // Bytes 6-7 are temperature; skip
    int16_t rawGX = static_cast<int16_t>((_i2cBuf[8]  << 8) | _i2cBuf[9]);
    int16_t rawGY = static_cast<int16_t>((_i2cBuf[10] << 8) | _i2cBuf[11]);
    int16_t rawGZ = static_cast<int16_t>((_i2cBuf[12] << 8) | _i2cBuf[13]);

    // --- Scale to physical units ---
    data.rawAccX  = static_cast<float>(rawAX) / Config::AccelScaleFactor;
    data.rawAccY  = static_cast<float>(rawAY) / Config::AccelScaleFactor;
    data.rawAccZ  = static_cast<float>(rawAZ) / Config::AccelScaleFactor;
    data.rawGyroX = static_cast<float>(rawGX) / Config::GyroScaleFactor;
    data.rawGyroY = static_cast<float>(rawGY) / Config::GyroScaleFactor;
    data.rawGyroZ = static_cast<float>(rawGZ) / Config::GyroScaleFactor;

    // --- Apply calibration offsets ---
    float accX  = data.rawAccX  - _accelOffsetX;
    float accY  = data.rawAccY  - _accelOffsetY;
    float gyroX = data.rawGyroX - _gyroBiasX;
    float gyroY = data.rawGyroY - _gyroBiasY;

    // --- Kalman filter update ---
    // Using accelerometer value directly as angle measurement input.
    // TODO: For full tilt-angle estimation replace accX/accY with
    //       atan2-derived angles if angular range requires it.
    data.filtX = _kalmanX.getAngle(accX, gyroX, data.dt);
    data.filtY = _kalmanY.getAngle(accY, gyroY, data.dt);

    data.valid = true;
    return data;
}

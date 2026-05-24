// Original by Kristian Lauszus, TKJ Electronics (GPL2).
// https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp

#pragma once

class Kalman
{
public:
    Kalman();

    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle);

    float getRate();

    // --- Filter tuning ---
    void setQangle(float Q_angle);
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

private:
    float Q_angle;   // Process noise variance - accelerometer
    float Q_bias;    // Process noise variance - gyro bias
    float R_measure; // Measurement noise variance

    float angle; // Current angle estimate (state[0])
    float bias;  // Current gyro bias estimate (state[1])
    float rate;  // Bias-corrected rate (output only, not a state)

    float P[2][2]; // 2x2 error covariance matrix
};


#ifndef KALMAM_H_
#define KALMAN_H_

#define DEFAULT_Q_ANGLE 0.001f
#define DEFAULT_Q_BIAS  0.003f
#define DEFAULT_R_MEASURE  0.03f

class Kalman{
    /* Kalman filter variables */
    float Q_angle = 0.0f; // Process noise variance for the accelerometer
    float Q_bias = 0.0f; // Process noise variance for the gyro bias
    float R_measure = 0.0f; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle = 0.0f; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias = 0.0f; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate = {0.0f}; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}}; // Error covariance matrix - This is a 2x2 matrix

public:
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);

    // Used to set angle, this should be set as the starting angle
    void setAngle(float newAngle) { angle = newAngle; }; 

    // Return the unbiased rate
    float getRate() { return rate; }; 

    /* These are used to tune the Kalman filter */
    void setQangle(float newQ_angle) { Q_angle = newQ_angle; };

    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in DEFAULT_Q_BIAS. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */    
    void setQbias(float newQ_bias) { Q_bias = newQ_bias; };
    
    void setRmeasure(float newR_measure) { R_measure = newR_measure; };

    float getQangle() { return Q_angle; };

    float getQbias() { return Q_bias; };

    float getRmeasure() { return R_measure; };

    Kalman();
    ~Kalman() = default;
};


#endif /*KALMAN_H_*/
#ifndef PID_H_
#define PID_H_

#include "Filters.h"

class PID
{
    #define DTERM_LPF_FREQ    60 //Hz
    #define DTERM_MIN        -300.0f
    #define DTERM_MAX         300.0f
    #define PID_GYRO_RATE_BUF_LENGTH 5

    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;

    float P = 0.0f;
    float I = 0.0f;
    float D = 0.0f;

    float i_sum = 0.0f;
    float out_min = 0.0f;
    float out_max = 0.0f;
    float prev_error = 0.0f;

    PT1Filter DTermLPFFilter;
    
    FirFilter ErrorFirFilter;
    float gyroRateBuf[PID_GYRO_RATE_BUF_LENGTH];
    float dtermCoeffs[PID_GYRO_RATE_BUF_LENGTH];

    float constrainf(float amt, float low, float high)
    {
        if (amt < low)
            return low;
        else if (amt > high)
            return high;
        else
            return amt;
    }

public:

    void Initialize(uint32_t Kp, uint32_t Ki, uint32_t Kd, uint16_t desiredRateHz, uint32_t out_max);
    float Compute(float setpoint, float measured, float dT);

    float Get_Error()
    {
        return prev_error;
    }

    float Get_P()
    {
        return P;
    }
    
    float Get_I()
    {
        return I;
    }
    
    float Get_D()
    {
        return D;
    }

    void Set_Kp(uint32_t val)
    {
        Kp = (val * 1.0f) / 10;
    }
    
    void Set_Ki(uint32_t val)
    {
        Ki = (val * 1.0f) / 10;
    }
    
    void Set_Kd(uint32_t val)
    {
        Kd = (val * 1.0f) / 10;
    }    

    PID() = default;
    ~PID() = default;
};

#endif /*PID_H*/
#ifndef PID_H_
#define PID_H_

#include "Filters.h"

class PID
{
    #define DTERM_LPF_FREQ    60 //Hz
    #define DTERM_MIN        -300.0f
    #define DTERM_MAX         300.0f

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

    void Initialize(float Kp, float Ki, float Kd, uint16_t desiredRateHz, float out_max);
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

    PID() = default;
    ~PID() = default;
};

#endif /*PID_H*/
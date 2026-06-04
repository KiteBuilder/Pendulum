#include <cstdint>
#include "PID.h"

void PID::Initialize(uint32_t Kp, uint32_t Ki, uint32_t Kd, uint16_t desiredRateHz, uint32_t out_max)
{
    this->Kp = (Kp * 1.0f) / 10;
    this->Ki = (Ki * 1.0f) / 10;
    this->Kd = (Kd * 1.0f) / 10;

    this->out_min = (out_max * (-1.0f)) / 10;
    this->out_max = (out_max * 1.0f) / 10;

    //initialize DTerm LPF filter
    float loopTime_sec = 1.0f / (float)desiredRateHz;
    DTermLPFFilter.FilterInit(DTERM_LPF_FREQ, loopTime_sec);

    //simple d(t) - d(t-1) differentiator 
    dtermCoeffs[0] = 1.0f;
    dtermCoeffs[1] = -1.0f;
    dtermCoeffs[2] = 0.0f;
    dtermCoeffs[3] = 0.0f;
    dtermCoeffs[4] = 0.0f;

    ErrorFirFilter.firFilterInit(gyroRateBuf, PID_GYRO_RATE_BUF_LENGTH, dtermCoeffs);
}

float PID::Compute(float setpoint, float measured, float dT)
{
    float output;

    //calculate error
    float error = setpoint - measured;

    //Proportional term
    P = error * Kp;

    //Integral term (with anti-windup clamp)
    i_sum += error * dT;
    I = Ki * i_sum;

    //Derivative term
    float dterm_error = error;

    ErrorFirFilter.firFilterUpdate(dterm_error);
    dterm_error = ErrorFirFilter.firFilterApply();

    if (DTermLPFFilter.isFirstLoad())
    {
        DTermLPFFilter.FilterSetVal(dterm_error);
    }
    dterm_error = DTermLPFFilter.FilterApply(dterm_error);

    D = Kd * ((dterm_error /*- prev_error*/) / dT);
    D = constrainf(D, DTERM_MIN, DTERM_MAX);
    prev_error = error;//dterm_error;

    //Calculate output
    output = P + I + D;

    //Anti wind-up - stop integrating when saturated
    if (output > out_max) 
    {
        if (error > 0.0f)
        {
            i_sum -= error * dT;
        }

    } else if (output < out_min) 
    {
        if (error < 0.0f)
        {
            i_sum -= error * dT;
        }
    }

    output = constrainf(output, out_min, out_max);
    
    return output;
}

#include <cstdint>
#include "PID.h"

void PID::Initialize(float Kp, float Ki, float Kd, float out_max)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    this->out_min = -out_max;
    this->out_max = out_max;
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
    D = Kd * ((error - prev_error) / dT);

    output = P + I + D;

    //Anti wind-up - stop integrating when saturated
    if (output > out_max) 
    {
        output = out_max;

        if (error > 0.0f)
        {
            i_sum -= error * dT;
        }

    } else if (output < out_min) 
    {
        output = out_min;

        if (error < 0.0f)
        {
            i_sum -= error * dT;
        }
    }
    
    prev_error =  error;

    return output;
}

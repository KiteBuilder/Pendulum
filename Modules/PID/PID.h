#ifndef PID_H_
#define PID_H_

class PID
{
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float dT = 0.0f;
    float i_sum = 0.0f;
    float out_min = 0.0f;
    float out_max = 0.0f;
    float prev_error = 0.0f;

public:

    void Initialize(float Kp, float Ki, float Kd, float out_min, float out_max);
    float Compute(float setpoint, float measured, float dT);
    
    float Get_Error()
    {
        return prev_error;
    }

    PID() = default;
    ~PID() = default;
};

#endif /*PID_H*/
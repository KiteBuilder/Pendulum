#ifndef MIXER_H_
#define MIXER_H_

class Mixer
{
    float pid_out_min = 0.0f;
    float pid_out_max = 0.0f;
    float motor_out_min = 0.0f;
    float motor_out_max = 0.0f;
    float motor_out_range = 0.0f;

public:

    void Initialize(float pid_min, float pid_max, uint16_t motor_min, uint16_t motor_max);
    uint16_t Compute(float pid_out_val);

    Mixer() = default;
    ~Mixer() = default;
};

#endif /*MIXER_H*/
#ifndef MIXER_H_
#define MIXER_H_

class Mixer
{
    float pid_min = 0.0f;
    float pid_max = 0.0f;
    float pid_range = 0.0f;

    uint16_t motor_min = 0;
    uint16_t motor_max = 0;
    uint16_t motor_mid = 0;
    uint16_t motor_range = 0;

public:

    void Initialize(float pid_min, float pid_max, uint16_t dshot_min, uint16_t dshot_max, uint16_t dshot_mid);
    void Compute(float pidsum, uint16_t &throttle_left, uint16_t &throttle_right);

    uint16_t Get_MotorMid()
    {
        return motor_mid;
    }

    void Set_MotorMid(uint16_t dshot_mid)
    {
        motor_mid = dshot_mid;
    }

    Mixer() = default;
    ~Mixer() = default;

};

#endif /*MIXER_H*/
#include <cstdint>
#include "mixer.h"

void Mixer::Initialize(float pid_min, float pid_max, uint16_t motor_min, uint16_t motor_max)
{
    pid_out_min = pid_min;
    pid_out_max = pid_max;
    motor_out_min = motor_min;
    motor_out_max = motor_max;

    motor_out_range = motor_out_max - motor_out_min;
}

uint16_t Mixer::Compute(float pid_out_val)
{
    float motor_out = ((pid_out_max - pid_out_val) / (pid_out_max - pid_out_min)) * motor_out_range;

    return (uint16_t)motor_out;
}
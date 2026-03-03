#include <cstdint>
#include <cmath>
#include "mixer.h"

void Mixer::Initialize(float pidsum_min, float pidsum_max, uint16_t dshot_min, uint16_t dshot_max)
{
    pid_min = pidsum_min;
    pid_max = pidsum_max;
    pid_range = pid_max - pid_min;

    motor_min = dshot_min;
    motor_max = dshot_max;
    motor_range = motor_max - motor_min;
}

void Mixer::Compute(float pidsum, uint16_t &throttle_left, uint16_t &throttle_right)
{
    float motor_out = motor_range;
    motor_out -= (motor_range * (pid_max - fabs(pidsum))) / pid_range;

    if (pidsum > 0)
    {
        throttle_left = motor_min - motor_out * 3;
        throttle_right = motor_min + motor_out;
    }
    else
    {
        throttle_left = motor_min + motor_out;
        throttle_right = motor_min - motor_out * 3;
    }
}
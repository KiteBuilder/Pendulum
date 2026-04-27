#include <cstdint>
#include <cmath>
#include "mixer.h"

void Mixer::Initialize(float pidsum_min, float pidsum_max, uint16_t dshot_min, uint16_t dshot_max, uint16_t dshot_mid)
{
    pid_min = pidsum_min;
    pid_max = pidsum_max;
    pid_range = pid_max - pid_min;

    motor_min = dshot_min;
    motor_max = dshot_max;
    motor_mid = dshot_mid;
    motor_range = motor_max - motor_min;
}

void Mixer::Compute(float pidsum, uint16_t &throttle_left, uint16_t &throttle_right)
{
    //convert pidsum from the pid_range to the motor_range scale
    uint16_t motor_out = motor_range;
    motor_out -= (uint16_t)(((motor_range * 1.0f) * (pid_max - fabs(pidsum))) / pid_range);
    uint16_t throttle_min, throttle_max;

    //calculate minimal throttle
    if (motor_mid > motor_out)
    {
        throttle_min = (motor_mid - motor_out);
        if (throttle_min < motor_min)
        {
            throttle_min =  motor_min;
        }
    }
    else
    {
        throttle_min = motor_min;
    }

    //calculate maximum throttle
    throttle_max = motor_mid + motor_out;
    if (throttle_max > motor_max)
    {
        throttle_max = motor_max;
    }

    //apply throttle value to motors
    if (pidsum > 0)
    {
        throttle_left = throttle_min;
        throttle_right = throttle_max;
    }
    else
    {
        throttle_left = throttle_max;
        throttle_right = throttle_min;
    }
}
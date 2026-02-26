
#ifndef MODEL_CONFIG_H_
#define MODEL_CONFIG_H_

#include "mpu_commands.h"

#define GYRO_RATE_HZ    1000 //1000 microseconds, 1000Hz
#define PID_LOOP_HZ     2000 //2000 microseconds, 500Hz
#define GYRO_LPF_TYPE   GYRO_LPF_256HZ
#define ACC_SCALE       INV_FSR_16G
#define GYRO_SCALE      INV_FSR_2000DPS
#define KALMAN_Q        200

#endif /*MODEL_CONFIG_H_*/

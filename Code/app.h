#ifndef APP_H_
#define APP_H_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void initialization(void);
void exec(void);

#define DEBUG_ENABLED

#define SET_POINT  (0.0f) //in Grad

#define PID_P   80
#define PID_I   10
#define PID_D   1

#define MAX_PID_P       100
#define MAX_PID_I       50
#define MAX_PID_D       30
#define MAX_ERROR       90 //in Grad
#define PIDSUM_MAX      (MAX_ERROR * MAX_PID_P)

#define DSHOT_MIN  300
#define DSHOT_MID  950
#define DSHOT_MAX  1600


#pragma pack(push,1)
struct config_t{	//write to flash
    uint32_t id;
    uint32_t Kp;
    uint32_t Ki;
    uint32_t Kd;
    uint32_t motor_mid;
};
#pragma pack(pop)


#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */

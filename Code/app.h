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
#define MAX_ERROR  (90.0f) //in Grad

#define PID_P      (12.0f) // 0 - 30
#define PID_I      (3.0f)
#define PID_D      (0.8f)

#define MAX_PID_P       (30.0f)
#define PID_OUT_MIN    (0.0f)
#define PID_OUT_MAX    (MAX_ERROR * MAX_PID_P)

#define MIN_THROAT  800
#define MID_THROAT  1000
#define MAX_THROAT  1600

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */

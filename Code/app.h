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

#define PID_P   (9.0f)
#define PID_I   (2.0f)
#define PID_D   (0.8f)

#define MAX_PID_P       (10.0f)
#define MAX_ERROR       (90.0f) //in Grad
#define PIDSUM_MAX     (MAX_ERROR * MAX_PID_P)
 
#define DSHOT_MIN  1100
#define DSHOT_MAX  1700

#ifdef __cplusplus
}
#endif

#endif /* APP_H_ */
